# Extended 5G-simulator based on NS-3 and 5G-LENA

This simulator is based on NS-3 and 5G-LENA module, enhancing the simulator with different features 
(indoor factory propagation loss model, multi-conectivity, etc.)

It also provides two example scripts:
- A distribution center scenario, where several logistics activities are emulated toestimate the performance of the different traffic profiles.
- Dual-connectivity scenario

## Contributor
Developed by David Segura Ramos (PhD student in the field of cellular communication at University of Málaga, Spain)

## Features
- A distribution center scenario, where several different logistics activities are done, and the communications of these activities are modeled and used to estimate the performance of the different traffic profiles
- 3GPP 38.901 [1] Indoor Factory propagation loss model in all its variants (InF-SH, InF-DH, InF-SL, InF-DL)
- BWs parts with reconfigurable bandwidth and resource allocation
- Dual-connectivity feature, with Packet Duplication approach for downlink transmissions [2]
- RRC Idle state with configurable inactivity timer
- 5G RRC Inactive state [3]

## Building project
To build this project, type the following command:
```shell
./waf configure --disable-python --disable-examples --disable-tests --enable-static --build-profile=optimized --disable-werror
```

## Getting started
We offer two example scripts, one for the logistics scenario used in [4]
and the other for dual-connectivity used in [2]. Both scripts are located in scratch folder.

To run these scripts, type the following command:
```shell
./waf --run scratch/logistics-scenario
```

```shell
./waf --run scratch/dual-connectivity
```

## Implementation detail
This project is based on ns-3-dev (commit 25e0d01d2883ae6da3c18a4d926f6b9aa2b99128) and 5G-LENA module v1.2.
It has been tested on the following platforms:
```
- macOS Sonoma
- Ubuntu 22.04
```

## How to cite this project
If you use this simulator, please cite the following references:
- Segura, D.; Khatib, E.J.; Barco, R. Dynamic Packet Duplication for Industrial URLLC. Sensors 2022, 22, 587. https://doi.org/10.3390/s22020587
- Segura, D., Khatib, E. J., Barco, R. Evaluation of Mobile Network Slicing in a Logistics Distribution Center. arXiv preprint arXiv:2212.12482

## Reference
- [1] Study on Channel Model for Frequencies from 0.5 to 100 GHz, document TR 38.901, V16.1.0, 3GPP.
- [2] Segura, D.; Khatib, E.J.; Barco, R. Dynamic Packet Duplication for Industrial URLLC. Sensors 2022, 22, 587. https://doi.org/10.3390/s22020587
- [3] NR; Radio Resource Control (RRC); Protocol specification, document TS 38.331, V16.10.0, 3GPP
- [4] Segura, D., Khatib, E. J., Barco, R. Evaluation of Mobile Network Slicing in a Logistics Distribution Center. arXiv preprint arXiv:2212.12482
