import zmq
import time
import threading

POLLER_TIMEOUT = 10 # milliseconds

##
# @brief A Space to maintain and stop threads with an override poll function
class ThreadSpace(threading.Thread):
    ##
    # @brief A Thread space only needs an exit signal and a lock
    def __init__(self):
        super().__init__()
        self.exit_signal = threading.Event()
        self.lock = threading.Lock()
    
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
        while True:
            self.poll()
            if self.exit_signal.isSet():
                print("Exiting thread")
                return

    ##
    # @brief Stop the thread
    def stop(self):
        self.lock.acquire()
        self.exit_signal.set()
        self.lock.release()

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
    def __init__(self, context, topic, get_data, address, period = 1): 
        super().__init__()
        self.socket = context.socket(zmq.PUB)

        self.period = period

        self.callback = get_data

        self.topic = topic

        self.sock_addr = address

        print(address)
        self.socket.bind(address)

    ##
    # @brief Publisher poll simply publishes using the get data callback and the topic as a prefix
    def poll(self):
        body = str(self.callback())
        self.socket.send_string(("%s:%s" % (self.topic, body)))
        time.sleep(self.period)


##
# @brief A Requesting thread space simply requests, once it gets the data, it waits period. I might change that
class RequesterThreadSpace(ThreadSpace):

    ##
    # @brief Initialize a requester thread space
    #
    # @param context The "global" context to create sockets from
    # @param period The period to wait after a request has been carried out
    # @param get_data The callback to get the message to request
    # @param recieve_request The callback to the recieved message
    # @param address The address to connect to, this can be changed using set_address
    def __init__(self, context, get_data, recieve_request, address, period = 1):
        super().__init__()
        self.socket = context.socket(zmq.REQ)
        self.period = period

        self.get_request = get_data
        self.recieve_request = recieve_request

        self.topic = topic
        self.sock_addr = address
        # Shouldn't connect if we do a concurrent request 
        # TODO Important:
        # I need to do a timing test to see if connecting every loop changes anything
        self.socket.connect(self.sock_addr)

    def poll(self):
        self.socket.send_string("%d %d" % (self.topic, self.get_request()))
        ret = self.socket.recv_string()
        self.recieve_request(ret)
        time.sleep(self.period)

    def set_address(self, address):
        self.socket.disconnect(self.sock_addr)
        self.sock_addr = address
        self.socket.connect(self.sock_addr)


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
    def __init__(self, context, callback, address, period = -1):
        super().__init__()
        self.socket = context.socket(zmq.REP)

        self.period = period

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
        if self.period > 0:
            time.sleep(self.period)

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
    def __init__(self, context, callback, address, topic="", period = -1):
        super().__init__()
        self.socket = context.socket(zmq.SUB)

        self.period = period
        self.callback = callback
        self.topic = topic
        self.sock_addr = address

        print(address)
        self.socket.connect(address)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.subscribe_to(topic)

    ##
    # @brief Poll a subscriber thread
    def poll(self):
        socks = dict(self.poller.poll(POLLER_TIMEOUT))
        if self.socket in socks and socks[self.socket] == zmq.POLLIN:
            message = self.socket.recv_string()
            topic, body = message.split(":")
            self.callback(body)
        if self.period != -1:
            time.sleep(self.period)

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
    def __init__(self, client = False, publisher = (True, "tcp://localhost:5555")):
        self.context = zmq.Context.instance()
        self.init_self_publisher(publisher[0], publisher[1])
        self.init_self_client(client)
        self.publishers = {}
        self.requesters = {}
        self.servers = {}
        self.subscribers = {}
        self.active = False

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
        for i in self.publishers.values():
            if i.is_alive():
                i.stop()
                i.join()
        for i in self.subscribers.values():
            if i.is_alive():
                i.stop()
                i.join()
        for i in self.servers.values():
            if i.is_alive():
                i.stop()
                i.join()
        for i in self.requesters.values():
            if i.is_alive():
                i.stop()
                i.join()
        self.active = False

    ##
    # @brief Create a concurrent publisher to use in this thread
    #
    # @param publisher The publisher to create a publisher with
    # @param address The address to bind to
    def init_self_publisher(self, publisher, address):
        if publisher:
            self.this_publisher = self.context.socket(zmq.PUB)
            self.this_publisher.bind(address)
        else:
            self.this_publisher = None

    ##
    # @brief Create a concurrent client in this thread 
    #
    # @param client The client to create in the thread 
    def init_self_client(self, client):
        self.this_client = self.context.socket(zmq.REQ) if client else None

    ##
    # @brief Publish concurrently 
    #
    # @param topic The topic to publish onto
    # @param message The message to send_string to this publisher
    def publish_concurrent(self, topic, message):
        if self.this_publisher == None:
            print("Error, no concurrent publisher")
        else:
            self.this_publisher.send_string("%d %d" % (topic, message))

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
        else:
            self.this_client.connect(address)
            self.this_client.send_string(message)
            val = self.this_client.recv_string()
            self.this_client.disconnect(address)
            return val

    def publish(self, sock_addr, topic, get_data, period, start_on_creation=False):
        if sock_addr not in self.publishers:
            self.publishers[sock_addr] = PublisherThreadSpace(context = self.context,\
                                                                        period = period,\
                                                                        topic = topic,\
                                                                        get_data = get_data, \
                                                                        address = sock_addr)
            if start_on_creation:
                self.publishers[sock_addr].start()
        else:
            print("Thread %s already in publishers" % (sock_addr))

    def cancel_periodic_publisher(self, sock_addr):
        if sock_addr in self.publishers:
            if self.publishers[sock_addr].is_alive():
                self.publishers[sock_addr].stop()
        else:
            print("%s does not exist" % (sock_addr))

    def subscribe(self, sock_addr, topic, callback, period = -1, start_on_creation = False):
        if sock_addr not in self.subscribers:
            self.subscribers[sock_addr] = SubscriberThreadSpace(context = self.context, \
                                                                topic = topic, \
                                                                callback = callback, \
                                                                address = sock_addr, \
                                                                period = period)
            if start_on_creation:
                self.subscribers[sock_addr].start()
        else:
            print("%s already exists in subscriber map" % (sock_addr))

    def cancel_periodic_subscriber(self, sock_addr):
        if sock_addr in self.subscribers:
            if self.subscribers[sock_addr].is_alive():
                self.subscribers[sock_addr].stop()
        else:
            print("%s does not exist" % (sock_addr))

    def request(self, address, destination, get_data, callback, period, start_on_creation = False):
        if address not in self.requesters:
            self.requesters[address] = RequesterThreadSpace(context = self.context, \
                                                            period = period, \
                                                            get_data = get_data, \
                                                            recieve_request = callback, \
                                                            address = address)
            if start_on_creation:
                self.requesters[address].start()
        else:
            print("%s already exists in requester map" % (address))
        

    def cancel_periodic_requester(self, sock_addr):
        if sock_addr in self.requesters:
            if self.requesters[sock_addr].is_alive():
                self.requesters[sock_addr].stop()
        else:
            print("%s does not exist" % (sock_addr))

    def serve(self, address, callback, period = -1, start_on_creation = False):
        if address not in self.servers:
            self.servers[address] = ServerThreadSpace(context = self.context, \
                                                      callback = callback, \
                                                      address = address, \
                                                      period = period)
            if start_on_creation:
                self.servers[address].start()
        else:
            print("%s already exists in server map" % (address))

    def terminate_server(self, address):
        if address in self.servers:
            if self.servers[address].is_alive():
                self.servers[address].stop()
        else:
            print("%s does not exist " % (address))
