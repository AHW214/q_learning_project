# Project Setup

1) Clone this repository.
    ```
    $ git clone https://github.com/AHW214/q_learning_project.git
    $ cd q_learning_project
    ```

2) Install `pipenv`, which this project uses for local package management.
    ```
    $ pip3 install pipenv
    ```

3) Install the packages used by this project (specified in [`Pipfile`](../Pipfile)).
    ```
    $ pipenv install
    ```

    This may appear to hang for a bit. It shouldn't be indefinite, though if it
    happens to be, contact us and we will help debug.

4) In one terminal, run the launch file for setting up the project environment.
    ```
    $ roslaunch q_learning_project setup.launch
    ```

5) Unpause time in Gazebo.

6) In a second terminal, run the launch file for starting the various ROS nodes
   within the virtual python environment.
    ```
    $ pipenv run roslaunch q_learning_project run.launch
    ```
