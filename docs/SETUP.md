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

4) In one terminal, run the launch file for setting up the project environment.
    ```
    $ roslaunch q_learning_project setup.launch
    ```

5) In a second terminal, launch a shell in the virtual environment containing
    the installed packages.
    ```
    $ pipenv shell
    ```

6) In the second terminal, run the launch file for starting the various ROS nodes.
    ```
    $ roslaunch q_learning_project run.launch
    ```
