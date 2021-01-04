# Adversarial Chase and Run Cars Gym

Adversarial Chase and Run Cars Gym is a **gym environment**, to test and develop algorithms related to **Multi Agent Systems**, especially those related to Multi-Agent Reinforcement Learning. This was done under the Robotics Research Group (RoboReG) to try and learn emergent behavior between agents competing against each other via Reinforcment learning and how it could generate new control strategies.

The gym environment was built in the physics simulator **[PyBullet](https://pybullet.org/) - a python module for physics simulations of robots**, because of its easy usage and integration with RL frameworks like **Stable Baselines**. Work is currently being done on training the agents to compete via Multi Agent Reinforcement Learning.

## Installation Guidelines

This gym environment can be installed by following these steps:

0. Although not compulsory, we strongly recommend creating a virtual environment specific to this project. This will help in package management and for decluttering your workspace. A simple way to create a virtual environment is as follows:

   ~~~bash
   python3 -m venv <Env_Name>
   ~~~

   Activation and deactivation of the virtual environment, will be done as specified [here](https://docs.python.org/3/library/venv.html). Scroll down to the table where the activation method for various operating systems is provided. Deactivation, in most cases, can be done by simply typing deactivate while being in in the virtual environment.

1. Once you activate your virtual environment, you will have to install the various dependencies of this project. We have simplified this process for you. Just follow the following steps:
   * Download/Clone this repository on to your local machine.
   * Navigate to the root folder of this repository through your terminal.
   * Execute the following command in your terminal.

      ~~~bash
      pip install -e adversarial-gym
      ~~~
   * Now, the environment can be created by calling this function in the python file:
      ~~~
      env = gym.make("adversarial_cars-v0")
      ~~~

In case there are problems with the PyBullet installation, you can refer to this [guide](https://github.com/Robotics-Club-IIT-BHU/Robo-Summer-Camp-20/blob/master/Part1/Subpart%201/README.md).

      
## Made and maintained by

<table>
   <td align="center">
      <a href="https://github.com/Terabyte17">
         <img src="https://avatars1.githubusercontent.com/u/60649571?s=400&u=e8e56b7d722ad82052f836ca929c79216144e425&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Yash Sahijwani</b>
         </sub>
      </a>
      <br />
   </td>
   <td align="center">
      <a href="https://github.com/aksayushx">
         <img src="https://avatars2.githubusercontent.com/u/55887638?s=400&u=bb089c514c11a7c371da30247f738f6a9489b168&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Ayush Kumar Shaw</b>
         </sub>
      </a>
      <br />
   </td>
   <td align="center">
      <a href="https://github.com/Raghav-Soni">
         <img src="https://avatars3.githubusercontent.com/u/60649723?s=460&v=4" width="100px;" alt=""/>
         <br />
         <sub>
            <b>Raghav Soni</b>
         </sub>
      </a>
      <br />
   </td>
</table>
