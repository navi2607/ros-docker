# ros-docker

This app uses docker-compose to deploy multiple ROS nodes. docker-compose uses 
a docker-compose.yml configuration file where all the nodes are defined. This 
project has a default Dockerfile and a default docker-compose.yml file. 

### Dockerfile

The default Dockerfile creates a image that has Ubuntu 18.04, Python 3.8 and 
ROS Melodic Morenia.

### docker-compose.yml
The docker-compose file deploys three nodes. A ros-master, ros-publisher and  
ros-listener node.

### App description
The nodes created with docker-compose run a Python 3.8 app that is in the ./scripts
folder. The publisher node runs only code specific for the publisher and the listener
runs only the code that is specific for the listener. Both of them use a producer/listener
configuration file in ./scripts/publisher for the publisher and ./scripts/listener for the 
listener. 

When the app starts the publisher starts to produce messages from that are pullded 
from the ./scripts/msg.yml file. The listener listens for any incoming messages.

Both the publisher and listener save their output to a log file that can be accessed 
in the ./output folder. (e.g publisher-output.txt and listener-output.txt) 

### Run app
```
# Run the containers as a deamon
sudo docker-compose up -d

# Stop the containers
sudo docker-compose down
```