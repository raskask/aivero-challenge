FROM rust:1.46 as builder

WORKDIR /user/src/app
COPY . .

RUN cargo install --path .

FROM debian:buster-slim

RUN apt update && apt install -y libssl1.1 libssl-dev && rm -rf /var/lib/apt/lists/*
COPY --from=builder /usr/local/cargo/bin/random_number_generator /usr/local/bin/random_number_generator


CMD ["random_number_generator"]