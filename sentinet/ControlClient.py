import zmq
import time
import threading
import signal
import sys
from sentinet.common import *

POLLER_TIMEOUT = 3000 # milliseconds
REQUEST_RETRIES = 3

##
# @brief A Space to maintain and stop threads with an override poll function
class ThreadSpace(threading.Thread):
    ##
    # @brief A Thread space only needs an exit signal and a lock
    def __init__(self, period, exit):
        super().__init__()

        # An event to kill the thread
        self.exit_signal = exit
        self.lock = threading.Lock()
        self.poll_period = period # seconds

        # Threads will die when main process 
        # exits if we forget to call event.set
        self.daemon = True
    
    ##
    # @brief To be overridden, the function to be executed continuously
    # @note it is up to the implimentation to make sure periods are met, otherwise, this
    # is done continusously
    def poll(self):
        print("Nothing to poll")

    ##
    # @brief The actual thread function (an overriden function)
    # @return When exit signal is set
    def run(self):
        while not self.exit_signal.wait(self.poll_period):
            try:
                self.poll()
            except (KeyboardInterrupt, SystemExit):
                sys.exit()

##
# @brief A Publisher thread space simply publishes every period
class PublisherThreadSpace(ThreadSpace):

    ##
    # @brief Initialize publishers
    #
    # @param context The "global" context to create sockets from
    # @param period The period to publish
    # @param topic The topic to publish to
    # @param get_data A callback that returns a byte string to publish
    # @param address The address to bind the publisher to 
    def __init__(self, context, topic, get_data, address, exit, period = 1.0): 
        super().__init__(period, exit)
        self.socket = context.socket(zmq.PUB)
        self.callback = get_data
        self.topic = topic
        self.sock_addr = address
        self.socket.connect(address)

    ##
    # @brief Publisher poll simply publishes using the get data callback and the topic as a prefix
    def poll(self):
        body = self.callback()
        self.socket.send_multipart([self.topic.encode('utf-8'), body])


##
# @brief A Server thread that simply listens passively for client connections
class ServerThreadSpace(ThreadSpace):
    ##
    # @brief Initialize a server
    #
    # @param context The "global" context to create sockets from
    # @param callback A Str (str) function that takes a string and returns a string to publish
    # @param address The address to bind to
    # @param period The period to check. This is normally ignored, I will do some unit tests
    def __init__(self, context, callback, address, exit, period = 1.0):
        super().__init__(period, exit)
        self.socket = context.socket(zmq.REP)

        self.callback = callback

        self.sock_addr = address
        self.socket.connect(address)
        
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

    ##
    # @brief Poll - simply scan all the sockets and check if we can get any data
    def poll(self):
        socks = dict(self.poller.poll(POLLER_TIMEOUT))
        if self.socket in socks and socks[self.socket] == zmq.POLLIN:
            message = self.socket.recv_string()
            val = self.callback(message)
            self.socket.send_string(val)

##
# @brief A Server thread that simply listens passively for client connections
class ServerThreadSpaceBound(ThreadSpace):
    ##
    # @brief Initialize a server
    #
    # @param context The "global" context to create sockets from
    # @param callback A Str (str) function that takes a string and returns a string to publish
    # @param address The address to bind to
    # @param period The period to check. This is normally ignored, I will do some unit tests
    def __init__(self, context, callback, address, exit, period = 1.0):
        super().__init__(period, exit)
        self.socket = context.socket(zmq.REP)

        self.callback = callback

        self.sock_addr = address
        self.socket.bind(address)
        
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

    ##
    # @brief Poll - simply scan all the sockets and check if we can get any data
    def poll(self):
        socks = dict(self.poller.poll(POLLER_TIMEOUT))
        if self.socket in socks and socks[self.socket] == zmq.POLLIN:
            message = self.socket.recv_string()
            val = self.callback(message)
            self.socket.send_string(val)

##
# @brief A Subscriber thread is passive and listens to its socket so that it can execute a callback
class SubscriberThreadSpace(ThreadSpace):
    ##
    # @brief Initialize a subscriber
    #
    # @param context The "global" context to create a socket from
    # @param topic The topic to subscribe to initially (if empty, subscribes to everything)
    # @param callback The callback when a subscriber recieves a message
    # @param address The address to connect to
    # @param period The polling period, usually not set, will do some unit tests
    def __init__(self, context, callback, address, exit, topic="", period = 1.0):
        super().__init__(period, exit)
        self.socket = context.socket(zmq.SUB)

        self.callback = callback
        self.topic = topic
        self.sock_addr = address

        self.socket.connect(address)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.subscribe_to(topic)

    ##
    # @brief Poll a subscriber thread
    def poll(self):
        socks = dict(self.poller.poll(POLLER_TIMEOUT))
        if self.socket in socks and socks[self.socket] == zmq.POLLIN:
            topic, body = self.socket.recv_multipart()
            self.callback(body)

    ##
    # @brief Subscribe to a certain topic
    #
    # @param topic The topic to subscribe to
    def subscribe_to(self, topic):
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)

##
# @brief A Control Client in python
class ControlClient:

    ##
    # @brief Initialize a control Client in python
    #
    # @param client If set to true, we have a concurrent requester
    # @param publisher If set to true, we have a concurrent publisher
    def __init__(self, address_in):
        self.context = zmq.Context.instance()

        # Initialize publisher and client
        self.this_publisher = None
        self.this_client = None

        # Dicts of managed sockets
        self.publishers = {}
        self.requesters = {}
        self.servers = {}
        self.subscribers = {}

        # Running
        self.active = False

        # Common exit signal for all threads
        self.exit_signal = threading.Event()

        self.bound_server = ServerThreadSpaceBound(context = self.context, \
                                                   callback = self.bound_server_callback, \
                                                   address = address_in,  \
                                                   exit = self.exit_signal,  \
                                                   period = 0.1)
        self.bound_server.start()

    def bound_server_callback(self, val):
        return val + "default"

    ##
    # @brief Starts all threads, a user can start individual threads, or just loop through and start all of them
    def start(self):
        for i in self.publishers.values():
            if not i.is_alive():
                i.start()
        for i in self.subscribers.values():
            if not i.is_alive():
                i.start()
        for i in self.requesters.values():
            if not i.is_alive():
                i.start()
        for i in self.servers.values():
            if not i.is_alive():
                i.start()
        self.active = True

    ##
    # @brief Stops all threads, a user can stop individual threads, or just loop through and stop all of them
    def quit(self):
        # Notify threads to quit
        self.exit_signal.set()

        for i in self.publishers.values():
            if i.is_alive():
                i.join()
        for i in self.subscribers.values():
            if i.is_alive():
                i.join()
        for i in self.servers.values():
            if i.is_alive():
                i.join()
        for i in self.requesters.values():
            if i.is_alive():
                i.join()

        self.active = False

    ##
    # @brief Create a concurrent publisher to use in this thread
    #
    # @param publisher The publisher to create a publisher with
    # @param address The address to bind to
    def init_self_publisher(self, address):
        self.this_publisher = self.context.socket(zmq.PUB)
        self.this_publisher.connect(address)

    ##
    # @brief Create a concurrent client in this thread 
    #
    # @param client The client to create in the thread 
    def init_self_client(self):
        self.this_client = self.context.socket(zmq.REQ)

    ##
    # @brief Publish concurrently 
    #
    # @param topic The topic to publish onto
    # @param message The message to send_string to this publisher
    def publish_concurrent(self, topic, message):
        if self.this_publisher == None:
            print("Error, no concurrent publisher")
        else:
            self.this_publisher.send_multipart([topic.encode('utf-8'), message.encode('utf-8')])

    ##
    # @brief Request concurrently
    #
    # @param address The address to connect to
    # @param message The message to send_string to the server
    #
    # @return The response if any from the server
    def request_concurrent(self, address, message):
        if self.this_client == None:
            print("Error, no concurrent client")
            return ""
        else:
            client = self.context.socket(zmq.REQ)
            client.connect(address)

            poll = zmq.Poller()
            poll.register(client, zmq.POLLIN)

            sequence = 0 
            retries_left = REQUEST_RETRIES
            while retries_left:

                # Our request in byte form
                if(type(message) == str):
                    request = message.encode('utf-8')
                else:
                    request = message

                # Attempt to send request
                client.send(request)

                # Wait until we dont expect a reply
                expect_reply = True
                print(address)
                while expect_reply:
                    # Poll the sockets
                    socks = dict(poll.poll(POLLER_TIMEOUT))
                    if socks.get(client) == zmq.POLLIN:
                        reply = client.recv()
                        if not reply:
                            break
                        retries_left = REQUEST_RETRIES
                        expect_reply = False
                        return reply
                    else:
                        print("W: No response from server, retrying...")
                        # Socket is confused. Close and remove it.
                        client.setsockopt(zmq.LINGER, 0)
                        client.close()
                        poll.unregister(client)
                        retries_left -= 1
                        if retries_left == 0:
                            print("E: Server seems to be offline, abandoning")
                            break
                        print("I: Reconnecting and resending (%s)" % request)
                        # Create new connection
                        client = self.context.socket(zmq.REQ)
                        client.connect(address)
                        poll.register(client, zmq.POLLIN)
                        client.send(request)
            return "Error".encode('utf-8')


    ##
    # @brief Equivalent to spin()
    #
    # @param pub_params Publish parameters
    def spin_publisher(self, pub: pub_params):
        self.publish(pub.sock_addr, pub.topic, pub.get_data, pub.period, pub.start_on_creation)

    def publish(self, sock_addr, topic, get_data, period, start_on_creation=False):
        if sock_addr not in self.publishers:
            self.publishers[sock_addr] = PublisherThreadSpace(context = self.context,\
                                                                        period = period,\
                                                                        topic = topic,\
                                                                        exit = self.exit_signal, \
                                                                        get_data = get_data, \
                                                                        address = sock_addr)
            if start_on_creation:
                self.publishers[sock_addr].start()
        else:
            print("Thread %s already in publishers" % (sock_addr))


    
    ##
    # @brief Same as spin
    #
    # @param sub_params Subscriber parameters
    def spin_subscriber(self, sub: sub_params):
        self.subscribe(sub.sock_addr, sub.topic, sub.callback, start_on_creation = sub.start_on_creation)

    def subscribe(self, sock_addr, topic, callback, period = -1, start_on_creation = False):
        if sock_addr not in self.subscribers:
            self.subscribers[sock_addr] = SubscriberThreadSpace(context = self.context, \
                                                                topic = topic, \
                                                                callback = callback, \
                                                                address = sock_addr, \
                                                                exit = self.exit_signal, \
                                                                period = period)
            if start_on_creation:
                self.subscribers[sock_addr].start()
        else:
            print("%s already exists in subscriber map" % (sock_addr))


    ##
    # @brief Same as spin
    #
    # @param serve_params Server parameters
    def spin_server(self, srv: serve_params):
        serve(srv.address, srv.callback, srv.start_on_creation)

    def serve(self, address, callback, period = -1, start_on_creation = False):
        if address not in self.servers:
            self.servers[address] = ServerThreadSpace(context = self.context, \
                                                      callback = callback, \
                                                      address = address, \
                                                      exit = self.exit_signal, \
                                                      period = period)
            if start_on_creation:
                self.servers[address].start()
        else:
            print("%s already exists in server map" % (address))
