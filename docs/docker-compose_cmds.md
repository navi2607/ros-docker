### Run container with docker-compose

```
docker-compose up -d
```

* up --> this parameter automatically deploys the services that are defined in the docker-compose.yml file
* -d --> optional argument to run the container as a deamon

If the container is started as a deamon run the following command to get the stdout logs:

``` 
docker-compose logs
```

### Stop container with docker-compose

```
docker-compose up -d  # Start the container as deamon  
docker-compose down  # Stop the container
```

### Restart container with docker-compose

```
docker-compose up -d  # Start the container as deamon  
docker-compose restart  # Restart the container
```
