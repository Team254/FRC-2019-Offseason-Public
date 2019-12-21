# FRC 2019 Offseason

Code for an FRC swerve drive with [Backlash's](https://www.team254.com/first/2019/) superstructure.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

- Java 11

### Installing

Clone this repository

```
$ git clone https://github.com/Team254/FRC-2019-Offseason-Public.git
```

Install all necessary tools: Gradle, Third-Party libraries, etc.

```
$ ./gradlew
```

If you're using IntelliJ, run the script to generate the IDEA project files, and then open in IntelliJ

```
$ ./gradlew idea
```


## Usage

To build, run 

```
$ ./gradlew build
```

To deploy to a connected roboRIO, run 

```
$ ./gradlew deploy
```

To see other Gradle tasks, run 

```
$ ./gradlew tasks
```