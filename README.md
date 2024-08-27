# About WRO 2024 FE Team Skibidi (SP Robotics)

This repository contains engineering materials of a self-driven vehicle's model participating in the [World Olympiad Robots](https://wro-association.org/)[Future Engineers competition](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf) in the 2024 season. This includes the source code, 3D CAD models and drawings, and team photos and vechical photos. 

## Content

* `t-photos` contains an offical photo and a funny/pose photo of the team members
* `v-photos` contains 6 photos of the vehicle from every side, from top and bottom
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Hardware

The hardware comprises of Raspberry Pi, a Raspberry Pi Global Shutter Global a few ultrasonic sensors for object collision.

In our [engineering journal](url), we detailed our decisions on why we picked certain components over the other. The following is a table with all the parts and components we used in the project.

## Recreating the project

1. **Fabricating the 3D models**
    * In the `models` folder, open `vehicle assembly v2.iam` in **AutoDesk Invetor** where you will see the modelled vechicle with all the parts.
    * Select the modelled that you would like to 3D print. Alternatively you may open `slicing` where the modelled for printing have already been exported
    * Open your 3D-printing software utility (e.g. UltiMaker Cura, Bambu Studio), config your print settings to fit your needs, and slice the models
2. **Assemblying the parts**
    * After printing the parts and ordering the components, assemble them according to the `vehicle assembly v2.iam` assembly file
    * Connect the components to each other with the header wires, power wires, and other connectors as necessary
3. **Preparing the Raspberry Pi**
    * Ensure that the Raspberry Pi is on the latest version
    * Raspberry Pi should read the camera and GPIO pins properly
    * Clone the repository, and enter the `src` folder
    * Download the dependcies using `pip install -r requirements.txt`
    * Run the code using the command `python3 main.py` (note: use `python` instead of `python3` yields a command not found error)

## Team Introduction

[Felix Isaac Lim](https://linkedin.com/in/felixisaac), [Chen Yun-Ting](https://www.linkedin.com/in/chenyun-ting/) and [John Tan](https://www.linkedin.com/in/zhttan/), are the members of [Singapore Polytechnic](https://sp.edu.sg)'s [Robotics Innovation & Technology Enterprise](https://https://www.sp.edu.sg/ccas/our-clubs/special-interests/sp-robotics-innovation-technology-enterprise) (SP RITE). Our team formed on 4<sup>th</sup> June of 2024, with one other member who quit the team early to focus on his academics — Yun-Ting volunteered to join our project as a mmeber afterwards.

<mark>**TODO: ** insert team photo here</mark>

We had about 10 weeks to; make a electronics component list and order the items, 3D design the car model and fabricate the parts, program the car, assemble integrate everything, test and refine. All within 10 weeks on top of school projects and test papers. Because of the tight timeline we had, [we took reference from the previous year](https://github.com/DelvinHo/WRO2023FutureEngineers-HexaVoid) we participated, and split up the work and did things in parallel. Zng Hiong and Yun-Ting continued to refine the model of the components and baseplate, and Felix focused on the programming aspects while we discussed how the car is going to work and which components to use. We both shared our work at times.

## License
This project is license under the [Apache License 2.0](https://github.com/FelixIsaac/WRO2024FutureEngineers-Team_SPRITE_Skibidi/blob/main/LICENSE). A permissive license whose main conditions require preservation of copyright and license notices. We provide an express grant of patent rights. Licensed works, modifications, and larger works may be distributed under different terms and without source code.
