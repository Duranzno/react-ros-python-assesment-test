<h1 align="center">Welcome to tech_aduran 👋</h1>
<p>
  <img alt="Version" src="https://img.shields.io/badge/version-0.0.1-blue.svg?cacheSeconds=2592000" />
  <a href="https://www.notion.so/duranzno/Ekumen-ROS-Technical-Interview-50fc1be5b9634e738c5f72743d9e9226" target="_blank">
    <img alt="Documentation" src="https://img.shields.io/badge/documentation-yes-brightgreen.svg" />
  </a>
  <a href="https://twitter.com/duranzno\_dev" target="_blank">
    <img alt="Twitter: duranzno\_dev" src="https://img.shields.io/twitter/follow/duranzno\_dev.svg?style=social" />
  </a>
</p>

> Technical Test for EkumenLabs implementing ROS and Web Technologies

## Usage

<!--
```sh
docker-compose build
docker-compose up
``` -->

<!-- ## Development -->

### For the ROS NODE we use a Docker image.

It will ask for a password to change the ownership of the src/ folder. From there you can use the ROS Terminal.
Inside the ROS Docker terminal tmux can be used or you the instance can be started with `docker exec -it ros-kinetic-dev-runned bash` or if ROS is installed in you OS it should not have any problem communicating with any `roscore` running in the container

```sh
cd backend/
./run.sh # Which will enter the docker container
./src/backend/entrypoint.sh
source devel/setup.bash
```

### For the Webapp made in React we use yarn locally, it will load the app but wont be usefull unless the ROS Node Docker image is running

```sh
cd client/
yarn install # If the repos are not installed
yarn start
```

## Author

👤 **Alejandro Durán**

- Website: https://duranzno.now.sh/
- Twitter: [@duranzno_dev](https://twitter.com/duranzno_dev)
- Github: [@duranzno](https://github.com/duranzno)
- LinkedIn: [@duranzno](https://linkedin.com/in/duranzno)

## 🤝 Contributing

Contributions, issues and feature requests are welcome!<br />Feel free to check [issues page](https://github.com/ekumenlabs/tech_aduran/issues).

## Show your support

Give a ⭐️ if this project helped you!

---

_This README was generated with ❤️ by [readme-md-generator](https://github.com/kefranabg/readme-md-generator)_
