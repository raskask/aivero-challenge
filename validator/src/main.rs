#[macro_use]
extern crate serde;

use async_std::{fs::File, io::WriteExt, prelude::*, sync::Mutex};
use circular_queue::CircularQueue;
use env_logger;
use futures::executor::LocalPool;
use lapin::{
    message::Delivery, options::*, types::FieldTable, Channel, Connection, ConnectionProperties,
    Result,
};
use log::{error, info};
use std::{thread, time};

const QUEUE_NAME: &str = "solution";
const RUNNING_MAX_WINDOW_SIZE: usize = 100;
const DEFAULT_RESULT_FILE_NAME: &str = "results/result.csv";

#[derive(Deserialize)]
struct Message {
    sequence_number: u64,
    rand: i64,
    running_max: i64,
}

struct MessageParser {}
impl MessageParser {
    fn parse_delivery(delivery: Delivery) -> serde_json::Result<Message> {
        serde_json::from_slice(&delivery.data)
    }
}

struct MessageValidator {
    queue: Mutex<CircularQueue<i64>>,
    result_file: Mutex<File>,
}
impl MessageValidator {
    fn get_result_file_path() -> String {
        std::env::var("RESULT_FILE_PATH").unwrap_or_else(|_| DEFAULT_RESULT_FILE_NAME.to_string())
    }

    async fn recreate_results_file() -> File {
        let path = Self::get_result_file_path();
        File::create(path)
            .await
            .expect("Could not create output file")
    }

    fn csv_format<TU: ToString, TI: ToString, TB: ToString>(
        seq_no: TU,
        rand: TI,
        correct_solution: TI,
        testee_solution: TI,
        correct: TB,
    ) -> String {
        format!(
            "{},{},{},{},{}\n",
            seq_no.to_string(),
            rand.to_string(),
            correct_solution.to_string(),
            testee_solution.to_string(),
            correct.to_string()
        )
    }

    pub async fn new() -> Self {
        let mut file = Self::recreate_results_file().await;
        file.write_all(
            Self::csv_format(
                "seq",
                "rand",
                "correct_solution",
                "testee_solution",
                "correct",
            )
            .as_bytes(),
        )
        .await
        .expect("Could not write CSV header");

        Self {
            queue: Mutex::new(CircularQueue::with_capacity(RUNNING_MAX_WINDOW_SIZE)),
            result_file: Mutex::new(file),
        }
    }

    pub async fn validate(&self, message: &Message) -> bool {
        let mut q = self.queue.lock().await;
        q.push(message.rand);
        let max = q.iter().max().unwrap();

        message.running_max == *max
    }

    pub async fn dump_line(&self, message: &Message, correct: bool) {
        let csv = Self::csv_format(
            message.sequence_number,
            message.rand,
            *self.queue.lock().await.iter().max().unwrap(),
            message.running_max,
            correct,
        );
        let mut r_file = self.result_file.lock().await;
        r_file
            .write_all(csv.as_bytes())
            .await
            .expect("Could not dump CSV");
        r_file.flush().await.expect("Could not flush result file");
    }
}

async fn create_consumer(addr: String) -> lapin::Result<Channel> {
    let mut conn = Connection::connect(
        &addr,
        ConnectionProperties::default().with_default_executor(8),
    )
    .await;
    while conn.is_err() {
        info!("Couldn't connect, retrying in 5 s.");
        thread::sleep(time::Duration::from_secs(5));
        conn = Connection::connect(
            &addr,
            ConnectionProperties::default().with_default_executor(8),
        )
        .await;
    }

    info!("CONNECTED");

    let pub_channel = conn.unwrap().create_channel().await?;

    let queue = pub_channel
        .queue_declare(
            QUEUE_NAME,
            QueueDeclareOptions::default(),
            FieldTable::default(),
        )
        .await?;
    info!("Declared queue {:?}", queue);
    Ok(pub_channel)
}

async fn run_consumer(channel: Channel) -> lapin::Result<()> {
    let msg_validator = MessageValidator::new().await;

    loop {
        let consumer = channel
            .basic_consume(
                QUEUE_NAME,
                "validate_consumer",
                BasicConsumeOptions::default(),
                FieldTable::default(),
            )
            .await
            .expect("Failed to get consumer");

        for delivery in consumer {
            // info!("received message: {:?}", delivery);
            if let Ok((channel, delivery)) = delivery {
                channel
                    .basic_ack(delivery.delivery_tag, BasicAckOptions::default())
                    .await
                    .expect("basic_ack");

                match MessageParser::parse_delivery(delivery) {
                    Ok(msg) => {
                        let correct = msg_validator.validate(&msg).await;
                        msg_validator.dump_line(&msg, correct).await;
                    }
                    Err(e) => {
                        error!("Message did not have the correct format: {:?}", e);
                    }
                }
            }
        }
    }
}

fn main() -> Result<()> {
    if std::env::var("RUST_LOG").is_err() {
        std::env::set_var("RUST_LOG", "info");
    }
    env_logger::init();

    let addr = std::env::var("AMQP_ADDR").unwrap_or_else(|_| "amqp://0.0.0.0:5672/%2f".into());
    LocalPool::new().run_until(async {
        let channel = create_consumer(addr).await?;
        run_consumer(channel).await?;
        Ok(())
    })
}
