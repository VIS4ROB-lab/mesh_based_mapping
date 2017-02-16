# Code and Dataset of the paper "Real-Time Mesh-based Scene Estimation for Aerial Inspection" - IROS 2016
Real-Time Mesh-based Scene Estimation for Aerial Inspection
### Video
<a href="https://www.youtube.com/embed/LvmBjMvmZKA" target="_blank"><img src="http://img.youtube.com/vi/LvmBjMvmZKA/0.jpg" 
alt="Mesh" width="240" height="180" border="10" /></a>

### Related Publication:
 If you use our Code or Datasets in a scientific work, please cite the following publication:
 
 Lucas Teixeira and Margarita Chli, "Real-Time Mesh-based Scene Estimation for Aerial Inspection", in Proceedings of the IEEE/RSJ Conference on Intelligent Robots and Systems (IROS), 2016.
 
```
 @inproceedings{Teixeira:etal:IROS2016,
title	= {{Real-Time Mesh-based Scene Estimation for Aerial Inspection}},
author	= {Lucas Teixeira and Margarita Chli},
booktitle	= {Proceedings of the {IEEE/RSJ} Conference on Intelligent Robots and Systems({IROS})},
year	= {2016}
}
```

## Qualitative Result
You can reproduce our qualitative result using this bagfile. You can also visualize the point-cloud on the RViz software together with the laserscan of the facade.


## Code

The code that we developed is a extention of OKVIS [link]. On this repository you can find the part of the code that implement the algorithm described in the our paper. You have to adapt in order to with your code, but it is a fairly simple implementation. There are small improviments and adaptation, mainly in the rasterization part. We recomend instead of use this code, you should run the code already integrate on OKVIS. We will give the instructions below. 

### License
The original OKVIS is BSD, but we use two libraries that are not. Fade2D is a commercial software that can be used for research proposes for free. Please check their website <http://www.geom.at/> . The new rasterization code is inpired by the Scratchapixel.com's Tutorial and they request GPLv3 license. We will be working to remove this dependence in the future but ,in summary, by now, this code only can be used for Research and under GPLv3 license. 




## Datasets

### Aerial 1

Bagfile - https://drive.google.com/open?id=0B82ekrhU9sDmTTdIeFJXTlBBLVE

<a href="https://www.youtube.com/embed/SA4KoRjvx04" target="_blank"><img src="http://img.youtube.com/vi/SA4KoRjvx04/0.jpg" 
alt="Aerial 1" width="200"  border="10" /></a> 


### Aerial 2

Bagfile - https://drive.google.com/open?id=0B82ekrhU9sDmNjZiMTUxUWlHcnc

<a href="https://www.youtube.com/embed/FEQiClIlLZI" target="_blank"><img src="http://img.youtube.com/vi/FEQiClIlLZI/0.jpg" 
alt="Aerial 2" width="200"  border="10" /></a> 

### Aerial 3

Bagfile - https://drive.google.com/open?id=0B82ekrhU9sDmOUkzX2xrMWRSMEE

<a href="https://www.youtube.com/embed/HLIJ59BRaBo" target="_blank"><img src="http://img.youtube.com/vi/HLIJ59BRaBo/0.jpg" 
alt="Aerial 3" width="200"  border="10" /></a> 

### Ground 

Bagfile   - https://drive.google.com/open?id=0B82ekrhU9sDmTjVweklrNGdJTjA

<a href="https://www.youtube.com/embed/a-ITwYMPzZs" target="_blank"><img src="http://img.youtube.com/vi/a-ITwYMPzZs/0.jpg" 
alt="Ground" width="200"  border="10" /></a> 













