import pika
import json
import numpy
 print('Yes!')
connection = pika.BlockingConnection(pika.ConnectionParameters(host = host))
channel = connection.channel()

channel.queue_declare(queue='rand')


def callback(ch, method, properties, body):
        count= 1
        lst=[]
        if count < 100:
            count = count+1
            lst.append(body["running_max"])
        else: 
            sequence_number = body["sequence_number"]
            rand = body["rand"]
            running_max = numpy.maximum.accumulate(lst)
            dictionary = {
                "sequence_number": sequence_number,
                "rand": rand,
                "running_max": running_max
            }
            solution = json.dumps(dictionary)
            break

    channel.basic_consume(queue='rand', on_message_callback=callback, auto_ack=True)

channel.basic_publish(exchange='',
                      routing_key='solution',
                      body= solution)