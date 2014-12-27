# ShapeClip

## Towards Rapid Prototyping with Shape-Changing Displays for Designers

ShapeClip is a modular tool capable of transforming any computer screen into a z-actuating shape-changing display. This enables designers to produce dynamic physical forms by simply ‘clipping’ actuators onto screens.

ShapeClip displays are portable, scalable, fault-tolerant, and support runtime actuator re-arrangement. Users are not required to have knowledge of electronics or programming, and can develop motion designs with presentation software, image editors, and for those who require more control, web-standard interfaces.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=5YozIqFVEgY
" target="_blank"><img src="http://img.youtube.com/vi/5YozIqFVEgY/0.jpg" 
alt="ShapeClip" width="240" height="180" border="10" /></a>

## Getting Started

There are 4 main software packages included here:

1. [Clip Firmware](../blob/master/00_Firmware/Firmware/Firmware.ino)
2. [Websocket/Serial Bridge](../blob/master/10_Driver/wsserial)
3. [SUR40 Clip Detector (OpenCV)](../blob/master/15_Middleware/ShapeClipDetector)
4. [Javascript API](../blob/master/10_Driver/ShapeClipAPI.js)
5. [Web-based Sample Apps](../blob/master/20_Applications)

In terms of software that can be used to create motion designs, ShapeClip is not restricted to any particular software package. You are free to work with the tools you know.

To get started you will need to get your hands on some ShapeClip hardware.To do that, you can view our design documents in the 01_Hardware folder.

![alt text](http://i.imgur.com/Qs7ZSov.png "ShapeClip mkIII Board")


## Research

This work was conducted as part of the GHOST project (http://ghostfet-prod.cs.bris.ac.uk/) to further the understanding of shape-changing computer interfaces.

You can view our paper (CHI2015) online here. To cite the paper, use the following citation: ...

![alt text](http://www.ghost-fet.com/wordpress/wp-content/uploads/2012/11/chosen_logo_web.png "GHOST Logo")

## License

Copyright (c) Lancaster University, GHOST Project

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
