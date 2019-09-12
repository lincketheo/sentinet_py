# How to checkout / create / merge a branch

This project is restricted on master and release. You will not be able to commit to these branches. In order to make changes to the project, create a new branch

```bash
  $ git checkout -b my_new_branch
  $ git push -u origin my_new_branch

  ... Do stuff - add changes

  $ git add <specific files>
  $ git status
  $ git commit -m "my_new_branch: added changes"
  $ git push
```



## IMPORTANT
Every time you make changes, you need to rebase from master. To see why, remember master is recieving new changes all the time. If you push with an old master, this might cause merge conflicts
``` bash
  $ git fetch # not necessary, but I'm showing this here because it's useful to update your branches
  $ git pull --rebase
  $ git rebase origin/master
```




# Build instructions

### Installing dependencies
```bash
$ sudo apt install cmake make curl libcurl4-gnutls-dev autoconf automake libtool g++ unzip libzmq3-dev
# or just
$ make install-deps # in SentiNet home dir

# Install yaml-cpp Note that this will be depreciated as I will include yaml cpp in third party
$ git clone https://github.com/jbeder/yaml-cpp.git yaml-cpp
$ cd yaml-cpp
$ mkdir build
$ cd build 
$ cmake -DBUILD_SHARED_LIBS=ON ..
$ make
$ make install

#Install protobuf, I might also add this as a third party
https://github.com/protocolbuffers/protobuf/blob/master/src/README.md
# either get a release here: https://github.com/protocolbuffers/protobuf/releases/tag/v3.9.1
# or clone it using git
$ git clone https://github.com/protocolbuffers/protobuf.git
$ git submodule update --init --recursive
$ ./autogen.sh
$ ./configure
$ make
$ make check
$ sudo make install
$ sudo ldconfig
```
  
## To build
``` bash 
$ git clone https://github.com/tlincke125/SentiNet.git SentiNet
$ cd SeniNet
$ make #Note at this early of production, there are bound to be errors

# Some other targets

$ make install-deps # installs UBUNTU dependencies (using apt)
$ make format # formats code
$ make clean # removes build
$ make cmake # only reads cmake stuff
$ make compile # runs cmake as well as compile (make)
$ make setup # sets up a few scripts that help development
```
 
Artifacts are placed in ./build/x86_64/bin
``` bash
# example running an artifact
$ ./build/x86_64/bin/<example name>
$ ./build/x86_64/bin/proxy # starts the proxy, There are a lot of things I need to fix on proxy, but for now its a seperate app
```

#  Directory structure

## cpp
This is the main directory, you can pretty much ignore everything else for now
***
*include*:
All the header files and framework stuff

Most important things

- ./cpp/include/framework/control

- ./cpp/include/networking/zmq

- ./cpp/include/framework/utils

- ./cpp/include/networking/curl

Everything else in include is just scratch mostly

*src*:

All the implimentation files

- ./cpp/src/libs - implimentation of the libraries

- ./cpp/src/impl - the executables that are compiled


*models*:
This directory houses all .proto files for protobuf to parse
There's one debug module here right now


# Tutorial

The pub sub framework for sentinet is very simple. Here is an example publisher subscriber (in the same process)

For more examples, see cpp/src/impl/examples

Of course, this could be in two seperate processes. One for a publisher and one for a subscriber, this is just demonstrating the strength of a single Control Client instance

This example uses publisher subscriber, but this can also be serve request (will add an example in the future, somewhat more complex here)


```cpp
#include "control/ZMQControlClient.hpp"

int main() {
  
  // Two demonstrate, this will be the value that we publish
  std::string value("Well hello there");

  // Publish params are nice packets of publisher data that we can  spin up
  publish_params a;
  // Assign it to a front end address on a broker
  a.broker_frontend = "tcp://localhost:5570";
  // Assign a topic
  a.topic = "Topic";
  // Assign a callback (this is in the form of std::string (void) so a regular function
  // will do just fine as well)
  a.get_data = [&value] (void) -> std::string {
    std::cout<<"Sending: "<<value<<std::endl;
    return value;
  };  
  // Set the period - defauts to 100 microseconds
  a.period = std::chrono::seconds(1);

  // Create a new subscriber param set
  subscribe_params b;
  // Attatch a callback
  b.callback = [] (std::string& val) -> void {
    std::cout<< "Recieved " << val << std::endl;
  };  
  // Assign a backend address
  b.socket_backend = "tcp://localhost:5571";
  // Assign a topic
  b.topic = "Topic";

  // Make a new control client
  auto cc = std::make_unique<ZMQControlClient>();

  // Now, all you have to do is spin up a new thingy
  cc->spin(b);
  cc->spin(a);

  // Sleep because this is asynchronous
  sleep(10);

  // ALWAYS quit
  cc->quit();

  return 0;
}
```


# DEPRECIATED USE THE ABOVE EXAMPLE USING PARAMS
```cpp
// Include Control Client (ZMQ Is just 1 implimentation of CC)
#include "networking/zmq/ZMQControlClient.hpp"

int main() {

  // Create a new ZCC instance
  auto a = std::make_unique<ZMQControlClient>();

  // Values to publish need to be ~global or else they are created everytime a publisher wants to publish
  std::string value_to_publish("Hi there");

  // parameters are (Address (soon won't be required), Topic, Callback function, period)
  /*
    a->publish("Topic name", value_to_publish); // In the future it will be this easy. This is like a 5 minute addition
   */
  a->publish("tcp://*:5555", "Topic name",
      [&]() -> std::string& {
        return value_to_publish;
        }, std::chrono::seconds(1));

  // parameters are (address to connect to (soon won't be required), Topic, callback)
  a->subscribe("tcp://localhost:5555", "Topic name", 
      [&](const std::string& value) -> void {
        std::cout << "Recieved "<<value<< " on topic Topic name"<<std::endl;
        });

  // Since this is inherently multithreaded, you need to have something else going on, a simple while(1) works too
  sleep(10);

  // ALWAYS quit. I'm thinking about adding this to the desctructor, but for now, quit, this terminates threads and stops callbacks
  a->quit();

  return 0;

}
```


## File Templates
After setting up with
```bash
$ make setup
```
If you use bash (not zsh), you can now execute generate. examples

```bash
$ generate hello.cpp
$ generate main.hpp
$ generate main.h
```

This just creates a nice template file



