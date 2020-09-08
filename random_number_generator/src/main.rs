use futures::executor::LocalPool;
use lapin::{options::*, publisher_confirm::Confirmation, types::FieldTable, BasicProperties, Connection, ConnectionProperties, Result, Channel};
use log::{info, debug};
use serde_json::json;
use std::time::Duration;
use async_std::task;
use env_logger;
use async_std::sync::Mutex;
use rand::{SeedableRng, Rng};

const QUEUE_NAME : &str = "rand";

struct MessageGenerator {
    sequence_counter: Mutex<u64>,
    random: Mutex<rand::rngs::StdRng>,
}
impl MessageGenerator {
    pub fn new() -> Self {
        Self {
            sequence_counter: Mutex::new(0),
            random: Mutex::new(rand::rngs::StdRng::from_entropy()),
        }
    }

    async fn get_and_update_sequence_counter(&self) -> u64 {
        let mut lock = self.sequence_counter.lock().await;
        let curr_val = *lock;
        *lock = curr_val + 1;
        curr_val
    }

    pub async fn get_message(&self) -> String {
        let seq_no = self.get_and_update_sequence_counter().await;
        let rand = self.random.lock().await.gen::<i64>();
        json!({
            "sequence_number": seq_no,
            "rand": rand
        }).to_string()
    }
}

async fn create_publisher(addr: String) -> lapin::Result<Channel> {
    let conn = Connection::connect(
        &addr,
        ConnectionProperties::default().with_default_executor(8),
    )
    .await?;

    info!("CONNECTED");

    let pub_channel = conn.create_channel().await?;

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

async fn run_publisher(channel: Channel) -> lapin::Result<()> {
    let msg_generator = MessageGenerator::new();
    let no_msgs = std::env::var("NUMBER_OF_MESSAGES").map(|n| n.parse::<u64>().expect("NUMBER_OF_MESSAGES is not a u64")).ok();
    let mut msg_counter = 0;

    while no_msgs.is_none() || no_msgs.unwrap() > msg_counter {
        let msg = msg_generator.get_message().await;
        let payload = msg.as_bytes();

        let confirm = channel
            .basic_publish(
                "",
        QUEUE_NAME,
                BasicPublishOptions::default(),
                payload.to_vec(),
                BasicProperties::default(),
            )
            .await?
            .await?;
        assert_eq!(confirm, Confirmation::NotRequested);
        debug!("Published message: {}", msg);

        msg_counter += 1;
        task::sleep(Duration::from_millis(200)).await;
    }

    Ok(())
}

fn main() -> Result<()> {
    if std::env::var("RUST_LOG").is_err() {
        std::env::set_var("RUST_LOG", "info");
    }
    env_logger::init();

    let addr = std::env::var("AMQP_ADDR").unwrap_or_else(|_| "amqp://0.0.0.0:5672/%2f".into());
    LocalPool::new().run_until(async {
        let channel = create_publisher(addr).await?;
        run_publisher(channel).await?;
        Ok(())
    })
}
