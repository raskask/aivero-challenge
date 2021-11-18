FROM python:3.8

ADD run_max.py .

WORKDIR /app/scr

COPY . .

RUN pip install pika numpy

CMD ["python", "run_max.py"]



