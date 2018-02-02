# Drone Vision

AR.Drone camera based tracking. In the actual phase it has 3 modes, a green laser pointer tracking, which can use any of the two cameras, and a led strip follower with the bottom camera (like a line follower robot, but with a drone).

## Getting Started

This will give you an overview of the project.

### ROS
The foundation of the project is based on [ROS(Robot Operating System)](http://www.ros.org/). It manages the communication of the algorithm on the PC with the drone, you can learn more about it on their [wiki page](http://wiki.ros.org/).
In the ROS environment there is one thing called packages, you can think of them as libraries, that make one's life easier. This project uses some of them, a interesting one being used is the [ardrone_autonomy package](https://github.com/AutonomyLab/ardrone_autonomy). It manages the AR.Drone SDK, making the control of the drone by software to be very easy.

### Python(OpenCV)
Since this is a camera based tracking project it needs to do image processing, which can be quite easy with [OpenCV](https://opencv.org/). The image processing algorithm it is using right now is based on a color tracking approach. It creates a mask based on a RGB treshold then applies it on the original image, as a result we have only the green dot. As a final step it recognizes the green dot as a circle and sends the x,y position to the control algorithm.

### Control(PID)
For the tracking it uses a simple [PID controller](https://en.wikipedia.org/wiki/PID_controller). The constants are optimized for the AR.Drone we used, so you may need to change them (just one line of code in controller.py).


## Prerequisites

The things you need to run the project and how to install them. The project was developed and tested in ##Ubuntu 16.04 LTS##, if your operating system is different it may need some workarounds.


```
ROS
Python 2.7
OpenCV with Python bindings
```

### Installing

#### ROS



Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc
