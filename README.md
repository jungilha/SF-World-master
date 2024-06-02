# San Francisco World: Leveraging Structural Regularities for 3-DoF Visual Compass
This package provides a MATLAB implementation of the RA-L 2024 paper: "San Francisco World: Leveraging Structural Regularities for 3-DoF Visual Compass" for the purpose of research and study only.

![SF-World](https://github.com/jungilha/SF-World-master/blob/main/pipeline.png)


# 1. Goal
Our goal is to estimate 3-DoF camera rotation with respect to indoor structured environments with slopes.
The proposed method exploits line and plane primitives jointly to recognize the spatial regularities of orthogonal structured environments and the uniform inclination of slopes.
Lines from RGB images and surface normals from depth images are simultaneously used to perceive environmental regularities accurately and stably.
Our method tracks drift-free rotational motion while at least a single plane and a pair of lines parallel to the San Francisco world (MW) axes are visible. 

![SF-World](https://github.com/jungilha/SF-World-master/blob/main/result.png)


# 2. Prerequisites
This package is tested on the MATLAB R2023b on Windows 11 64-bit.


# 3. Usage
* Run SFworld_core/main_script_STSC_ARkit.m, which will give you the 3D motion estimation result. Enjoy! :)


# 4. Publications
The approach is described and used in the following publications:

* **San Francisco World: Leveraging Structural Regularities for 3-DoF Visual Compass** (Jungil Ham, Minji Kim, Suyoung Kang, Kyungdon Joo, Haoang Li, and Pyojin Kim), RA-L 2024.


# 5. License
The package is licensed under the MIT License, see http://opensource.org/licenses/MIT.

      author = {Kim, Pyojin and Coltin, Brian and Kim, H Jin},
      title = {Low-Drift Visual Odometry in Structured Environments by Decoupling Rotational and Translational Motion},
      year = {2018},
      booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
     }
