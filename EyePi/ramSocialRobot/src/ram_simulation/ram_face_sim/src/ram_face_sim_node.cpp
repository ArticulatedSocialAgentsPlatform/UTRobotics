#include <ros/ros.h>
#include <ram_animation_msgs/Face.h>

#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#define eyeWidth    7
#define eyeOriginRX 6
#define eyeOriginRY 7
#define eyeOriginLX 17
#define eyeOriginLY 7
#define eyeSize     3
#define width     24
#define height 16
#define swap(a, b) { uint16_t t = a; a = b; b = t; }

int emotionState;
int emotionLevel;
int xPosition;
int yPosition;
int screenbrightness;

float map(float value, float istart, float istop, float ostart, float ostop) {
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}


uint8_t screenoutput[width][height];

void drawPixel(uint8_t i, uint8_t j, uint8_t color) {
	screenoutput[i][j] = color;
}

void clrPixel(uint8_t i, uint8_t j) {
	screenoutput[i][j] = 0;
}

void setPixel(uint8_t i, uint8_t j) {
	screenoutput[i][j] = 1;
}




// fill a rectangle
void fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
uint8_t color) {
	for (uint8_t i=x; i<x+w; i++) {
		for (uint8_t j=y; j<y+h; j++) {
			drawPixel(i, j, color);
		}
	}
}

// bresenham's algorithm - thx wikpedia
void drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1,
uint8_t color) {
	uint16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}
	uint16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);
	int16_t err = dx / 2;
	int16_t ystep;
	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;}
	for (; x0<=x1; x0++) {
		if (steep) {
			drawPixel(y0, x0, color);
		} else {
			drawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

// fill a circle
void fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;
	drawLine(x0, y0-r, x0, y0+r+1, color);
	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		drawLine(x0+x, y0-y, x0+x, y0+y+1, color);
		drawLine(x0-x, y0-y, x0-x, y0+y+1, color);
		drawLine(x0+y, y0-x, x0+y, y0+x+1, color);
		drawLine(x0-y, y0-x, x0-y, y0+x+1, color);
	}
}

void clrLine(int x1, int y1, int x2, int y2){
	for(int i=x1; i<=x2; i++){
		double y = (double)((double)((y2-y1)*(i-x1))/(x2-x1))+y1;
		if(y - (int)y > 0.1) y = y + 1.0;
		clrPixel(i, (int)y);
	}  
}

void setEyeBall(int x, int y){
	fillCircle(eyeOriginRX + x, eyeOriginRY + y, eyeSize, 1);   
	fillCircle(eyeOriginLX + x, eyeOriginLY + y, eyeSize, 1);
}

void writeMatrix(int emotionState, int emotionLevel, int xPosition, int yPosition) {

	// Start and End position for Line or Rect
	int xR1 = eyeOriginRX + eyeWidth/2; //inside
	int xL1 = eyeOriginLX - eyeWidth/2; //inside
	int xR2 = eyeOriginRX - eyeWidth/2; //outside
	int xL2 = eyeOriginLX + eyeWidth/2; //outside

	//// eyeballs  
	switch(emotionState){
	case 0:  //Neutral
		fillRect(0, 0, width, height, 0); // start with blank screen
		setEyeBall(xPosition, yPosition);
		break;  

	case 1:  //Happy
		fillRect(0, 0, width, height, 0); // start with blank screen
		setEyeBall(xPosition, yPosition);
		fillCircle(eyeOriginRX + xPosition, eyeOriginRY + yPosition - emotionLevel + 8, 3, 0);  // clear circle below Right eye
		fillCircle(eyeOriginLX + xPosition, eyeOriginLY + yPosition - emotionLevel + 8, 3, 0);  // clear circle below Left  eye
		break; 

	case 2:  //Sleep
		fillRect(0, 0, width, height, 0); // start with blank screen
		setEyeBall(xPosition, yPosition);
		// Arc
		for (int i=0; i<=emotionLevel; i++){  
			drawLine(0, eyeOriginRY - 5 + i + yPosition, width, eyeOriginRY - 5 + i + yPosition, 0);  // Upper arc
			drawLine(0, eyeOriginLY + 5 - i + yPosition, width, eyeOriginLY + 5 - i + yPosition, 0);  // Bottom top arc
		}
		break;

	case 3:  //Surprized 
		fillRect(0, 0, width, height, 0); // start with blank screen
		if(emotionLevel == 0) setEyeBall(xPosition, yPosition);
		// Change size
		else{
			fillCircle(eyeOriginRX + xPosition, eyeOriginRY + yPosition, emotionLevel + 1, 1);   
			fillCircle(eyeOriginLX + xPosition, eyeOriginLY + yPosition, emotionLevel + 1, 1);
		}
		break;

	case 4:  //Sadness
		fillRect(0, 0, width, height, 0); // start with blank screen
		setEyeBall(xPosition, yPosition);
		// Upper arc
		for (int i=0; i<=emotionLevel; i++){ 
			clrLine(xR2 + xPosition, eyeOriginRY - 4 + i + yPosition, xR1 + xPosition, eyeOriginRY - 7 + i + yPosition);
			clrLine(xL1 + xPosition, eyeOriginLY - 7 + i + yPosition, xL2 + xPosition, eyeOriginLY - 4 + i + yPosition); 
		}
		break; 

	case 5:  //Anger
		fillRect(0, 0, width, height, 0); // start with blank screen
		setEyeBall(xPosition, yPosition);
		for (int i=0; i<=emotionLevel; i++){\
			clrLine(xR2 + xPosition, eyeOriginRY - 7 + i + yPosition, xR1 + xPosition, eyeOriginRY - 4 + i + yPosition); // Upper arc
			clrLine(xL1 + xPosition, eyeOriginLY - 4 + i + yPosition, xL2 + xPosition, eyeOriginLY - 7 + i + yPosition); // Upper arc
			drawLine(0, eyeOriginRY + 7 - i  + yPosition, width, eyeOriginLY + 7 - i + yPosition, 0);    // Bottom arc  
		}
		break;

	case 6:  //Friendly
		fillRect(0, 0, width, height, 0); // start with blank screen
		setEyeBall(xPosition, yPosition);
		// Modify dots    
		if(emotionLevel > 0){
			setPixel(19 + xPosition, 11 + yPosition);
			clrPixel(16 + xPosition, 11 + yPosition);
			setPixel(15 + xPosition, 4  + yPosition);
			clrPixel(18 + xPosition, 4  + yPosition);

			setPixel(4 + xPosition, 11 + yPosition);
			clrPixel(7 + xPosition, 11 + yPosition);
			setPixel(8 + xPosition, 4  + yPosition);
			clrPixel(5 + xPosition, 4  + yPosition);
		}  
		// Upper arc
		if(emotionLevel > 1){
			for (int i=0; i<=emotionLevel; i++){
				clrLine(xR2 + xPosition, eyeOriginRY - 7 + i + yPosition, xR1 + xPosition, eyeOriginRY - 4 + i + yPosition); // Upper arc
				clrLine(xL1 + xPosition, eyeOriginLY - 4 + i + yPosition, xL2 + xPosition, eyeOriginLY - 7 + i + yPosition); 
			}
			drawLine(0, eyeOriginRY - 6 + emotionLevel + yPosition, width, eyeOriginRY - 6 + emotionLevel + yPosition, 0);  // Upper arc
		}
		break;

	default:
		setEyeBall(xPosition, yPosition);
		break;
	}
}

// A callback function. Executed each time a new pose message arrives
void poseMessageReceived(const ram_animation_msgs::Face& msg) {
	ROS_DEBUG_STREAM("Emotionstate: " << (uint8_t)msg.emotionState);
	emotionState=(uint8_t)msg.emotionState;
	xPosition = msg.eyePosX;
	yPosition = msg.eyePosY;
	screenbrightness = msg.screenBrightness;
	emotionLevel = msg.emotionLevel;
	writeMatrix(emotionState,map(emotionLevel,0,127,0,5), map(xPosition,0,127,3,-3), map(yPosition,0,127,4,-4));
}

int main(int argc, char **argv) {
	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "face_sim_node");
	ros::NodeHandle nh;

	// Create a subscriber object.
	ros::Subscriber sub = nh.subscribe("/ram/animation/generated/face", 1000, &poseMessageReceived);
	ros::Publisher gridPub = nh.advertise<nav_msgs::GridCells>("screengrid",1000);
	geometry_msgs::Point point;

	ros::Rate loop_rate(1000);

	while(ros::ok()) {
		nav_msgs::GridCells gridmsg;

		gridmsg.cell_width = 0.003;
		gridmsg.cell_height = 0.003;
		gridmsg.header.frame_id = "link4";
		gridmsg.header.stamp = ros::Time::now();
		
		for(int x = 0; x < width; x++)
		{
			for(int y = 0; y < height; y++)
			{
				if(screenoutput[x][y]==1)
				{
					point.x = float(x)/216-0.0535;	//Correct eye position
					point.y=float(y)/207-0.0345;	//Correct eye position
					point.z=0.083	;			//Correct eye position
					gridmsg.cells.push_back(point);
				}
			}
		}

		// Publish the message.	
		gridPub.publish(gridmsg);	
		ros::spinOnce();
		loop_rate.sleep();
	}
}
