# NDSSVehicleSec24

This repository contains the code use for the simulation for generating the results of the paper submitted at VehicleSec 24. 


# Installation
At first, please install conda from the website. Then, please install Sumo and Carla. We have tested with Sumo version 1.19 and Carla version 0.9.14. The installation instructions can be found in the following link.

https://sumo.dlr.de/docs/Installing/index.html

https://carla.readthedocs.io/en/latest/start_quickstart/

Please setup the conda environment by running the following command.

```
conda env create -n envName -f SumoCarla.yml
```

Then download/clone the repository and copy the files in the sumo folder where it is installed.

# Accident Results

This scenario was generated by combining Sybil and Bias Injection attack. We use the strategic attacker model (Sabbir et al., [2023](https://arxiv.org/abs/2305.16818)). As can be seen the injection of the attacks results in accidents in the absence of trust-ware robust CBF framework proposed in (Sabbir et al., [2023](https://arxiv.org/abs/2305.16818)), which is eliminated using our proposed robust event-triggered control and coordination framework.

We will need three terminals to run the scenarios. In the first terminal run the following:
```
./CarlaUE4.sh
```
Then in the second terminal, run the following:
```
python3 ./path_to_carla/config.py --map Town05
```
Finally, in the third terminal run the following two commands to run the simulation that results in accident:
```
python3 xmlconverter.py --dataset ./Dataset/Accident_dataset/dataset_init.xlsx
python3 run_synchronization_accident.py --accident 'True' examples/Town05.sumocfg
```

In order to run our porposed scheme that elimitates the accident run the following in the third terminal:
```
python3 xmlconverter.py --dataset ./Dataset/Accident_dataset/dataset_init.xlsx
python3 run_synchronization_accident.py --accident 'False' examples/Town05.sumocfg
```

# Attack Mitigation Results

The attack was generated by injecting a Sybil attack using the strategic attacker model in (Sabbir et al., [2023](https://arxiv.org/abs/2305.16818)) provided below. The aim of the attack was to create traffic holdup. The following video demonstrates the effect of the attack - creating jam in the network. 

https://github.com/anonymousCoder29/VehicleSec24/assets/155138234/3c1c3a4c-0d7a-4a6a-b745-44763f72f09d

Then, the result with the mitigation is shown in the video below. As can be seen, the traffic jam is eased with proposed mitigation scheme. 

https://github.com/anonymousCoder29/VehicleSec24/assets/155138234/a3f2a924-2a30-4d25-83ed-aa7cad7150ff

In order to run the scenarios we need three terminals. In the first terminal run the following:
```
./CarlaUE4.sh
```
Then in the second terminal, run the following:
```
python3 ./path_to_carla/config.py --map Town05
```
Finally, in the thrid terminal run the following two commands:
```
python3 xmlconverter.py --dataset ./Dataset/Mitigation_dataset/dataset_init.xlsx
```
https://github.com/anonymousCoder29/VehicleSec24/assets/155138234/92f9cb4a-9a17-420d-b1bc-72e778d40022


https://github.com/anonymousCoder29/VehicleSec24/assets/155138234/11b9f8fd-eecc-4899-8699-68191bbcf24f






