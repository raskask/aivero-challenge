FROM rust:1.46 as builder

WORKDIR /user/src/app
COPY . .

RUN cargo install --path .

FROM debian:buster-slim

RUN apt update && apt install -y libssl1.1 libssl-dev && rm -rf /var/lib/apt/lists/*
COPY --from=builder /usr/local/cargo/bin/validator /usr/local/bin/validator

CMD ["validator"]