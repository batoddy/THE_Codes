import threading


def print_a():
    while 1:
        print("Hello World!!!")


def print_c():
    while 1:
        print("Fuck World!!!")


t1 = threading.Thread(target=print_a, args=())
t2 = threading.Thread(target=print_c, args=())


t1.start()
t2.start()

t1.join()
t2.join()
