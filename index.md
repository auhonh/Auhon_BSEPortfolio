# Computer Vision Robot
My project is a Computer Vision Robot. It is a three-wheeled robot that utilizes ultrasonic sensors to detect a red ball and both turn and move toward it. It does this using a Raspberry Pi 4 where code is loaded and run, determining when each of the motors should run depending on the output of the ultrasonic sensors. 

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Auhon H. | Homestead High | Mechanical Engineering | Incoming Junior

**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
  
# Final Milestone - working robot + custom chassis (?)
For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/F7M7imOVGug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

# Second Milestone - code loaded onto raspberry pi (?)
For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone 

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/y3VAmNlER5Y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

# First Milestone - all wiring and assembly done (?)
For my first milestone, I finished wiring the full robot, as well as assembly of the chassis. As of right now, the wheels are connected to motors that are mounted on the plastic chassis. These motors, as well as a battery pack, are connected to an H-bridge. This H-bridge is connected to a Raspberry Pi 4, so the Raspberry Pi can control the motors. Additionally, the Raspberry Pi is connected to 3 ultrasonic sensors, which provide visual input. 

The current chassis is a generic plastic base, making it hard to mount all of the electrical components on the robot itself. As a result, I had to shorten and lengthen certain wires using wire strippers, soldering irons, and electrical tape in order to mount them properly. My plan to finish this mainly consists of writing code for the sensors to detect a red ball, and load it onto the Raspberry Pi. Additionally, I need to organize and modify the wires further. 

video here

# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resources to create professional schematic diagrams, though BSE recommends Tinkercad because it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

```c++
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
```

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.

# Starter Project
For my starter project, I created the Useless Box. This is a plain box with a single lever on the top of it. When this lever is flipped, an arm comes out of the box, flipping it back, and then going back into the box, reverting it to it's previous state. Additionally, an LED in the box turns green when the lever is flipped and the arm is moving towards it, red when the arm has hit the lever and is moving back into the box, and turns off when the arm is back in the box.

The major components of this project are a PCB, connecting a lever, an LED, a motor, and batteries. Additionally, there is a switch that is pressed by the arm when the box is closed, due the the unique shape of the arm. The LED is red when the lever is in its default state, green when the lever has been flipped by the user, but off when the switch is pressed. The batteries power the LED and motor, and the lever determines the state of the LED and the rotation direction of the motor. Surrounding the electronics is a black box, with a flush door on the top. The hoor has a hinge placed in a way so that the arm will push the door up when it flicks the lever and close the door after it moves back down.

When the lever is flicked by the user, the motor rotates, moving the arm towards the lever and away from the switch, causing the LED to turn green and the arm to rotate towards the lever. When the arms hits the lever, the LED changes to red and the motor starts rotating in the opposite direction, moving the arm back down until it reaches it default state. In this state, the switch is pressed, turning the LED off. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/Z2-qvxFV8pM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
