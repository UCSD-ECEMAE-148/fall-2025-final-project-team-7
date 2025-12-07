<div id="top"></div>
<h1 align="center">Autonomous Roadside Mechanic</h1>
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="images\UCSD-JSOE-LOGO.png" alt="Logo" width="432" height="108">
  </a>

## 

<h3>Team 7 </h3>
<h3>ECE/MAE 148 Final Project FA25</h3>
<p>
</p>
<img src="media\car.jpg?" width="605" height="501">
</div>

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#overview">Overview</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#demonstration">Demonstration</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#robot-design">Robot Design</a></li>
    <li><a href="#electrical-diagram">Electrical Diagram</a></li>
    <li><a href="#references">References</a></li>
  </ol>
  
## Team Members

<ul>
  <li>Alison Stosser - Mechanical Engineering</li>
  <li>Long-Giang Vu - MS Computer Science and Engineering</li>
  <li>Qiwen Xu - Electrical Computer Engineering</li>
  <li>Shubhan Mital - MS Electrical and Computer Engineering (MLDS)</li>
</ul>

## Overview
The goal of the "Autonomous Roadside Mechanic" is to identify and navigate a broken down car on the side of the road via hazard lights. The system uses two cameras, one “bird’s eye” camera mounted above the road which identifies there is a broken down vehicle via a blinking hazard light and one OAK-D camera mounted on the car to detect the car itself on the ground. Using ROS2, the “bird’s eye” camera sends a signal to activate the mechanic while the mechanic receives the signal to start driving on the track to navigate to and park behind the broken down car.

## What We Promised
### Must Have:
* Bird’s eye view camera that detects broken down stationary vehicle via hazard lights/no movement, sends signal to activate mechanic
* Receive “release” signal from bird’s eye view camera and drive track in single lane
* Detect hazard lights with OAK-D camera
* Stop behind the broken down car using LiDAR 

### Nice to Have:
* Avoid other cars/obstacles on the track
* Change lanes into emergency lane
* Ability to push the broken down car for a distance (simulating moving the car out of harm's way)


## Accomplishments
* tbd

## Demonstration
tbd

## Challenges
* tbd 
 
## Robot Design
<div align="center">
<img src="images\car-cad.png?" width="851" height="386">
</div>

## Electrical Diagram
<div align="center">
<img src="images\electrical-diagram.png?" width="581" height="365">
</div>
 
## References
* tbd
