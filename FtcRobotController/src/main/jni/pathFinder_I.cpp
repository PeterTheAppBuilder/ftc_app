#include "pathFinder_I.h"
#include <vector>
#include <string>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "p_Block.h"
#include <android/log.h>

JNIEXPORT jfloatArray JNICALL Java_ftc_pathfinder_pathFinderNative_calcPath
(JNIEnv * env, jclass jcl,jdouble _my_X, jdouble _my_Y, jdouble _target_X,
 jdouble _target_Y,jdouble _fieldSize, jlong _fieldMap ,jboolean _newPath){
	double my_X = _my_X;
	double my_Y = _my_Y;
	double target_X = _target_X;
	double target_Y = _target_Y;
	double fieldSize = _fieldSize;
    bool newPath = _newPath;

    //if there is a new path, reset our obstructedObstaclesLast variable
    if(newPath){
        obstructedObstaclesLast = false;
    }

    fieldMap = (Mat*)_fieldMap;
    createGrid();

	if(calcPath(my_X,my_Y,target_X,target_Y,fieldSize)){
        //dump the point positions into a simple float array
        std::vector<float> filteredPoints_x_y = std::vector<float>();
        for(int i = 0; i< m_filteredSteps.size();i++){
            filteredPoints_x_y.push_back(m_filteredSteps.at(i).x);
            filteredPoints_x_y.push_back(m_filteredSteps.at(i).y);
        }
        int sizeFloatArray = filteredPoints_x_y.size();

        //create a new float array to populate
        jfloatArray result;
        result = env->NewFloatArray(sizeFloatArray);
        // fill a temp structure to use to populate the java float array
        jfloat fillMe[sizeFloatArray];
        for(int i = 0;i<sizeFloatArray;i++){
            fillMe[i] = filteredPoints_x_y.at(i);
        }
        env->SetFloatArrayRegion(result,0,sizeFloatArray,fillMe);
        return result;
    }else{
        return env->NewFloatArray(0);
    }
}

JNIEXPORT void JNICALL Java_ftc_pathfinder_pathFinderNative_drawPath
(JNIEnv * env, jclass jcl,jlong _drawingImage, jint _scale){
    Mat& drawingImage = *(Mat*) _drawingImage;

    int scale = (int) _scale;

    for(int i = 0; i< m_grid.allBlocks.size();i++){
        if(m_grid.allBlocks.at(i).invalid){
            cv::Point tl = cv::Point(m_grid.allBlocks.at(i).x*scale,m_grid.allBlocks.at(i).y*scale);
            cv::Point br = cv::Point(tl.x + scale,tl.y+scale);

            cv::rectangle(drawingImage,tl,br,Scalar(255,255,255),-1);
        }
        if(m_grid.allBlocks.at(i).horizontalOnly){
            cv::Point tl = cv::Point(m_grid.allBlocks.at(i).x*scale,m_grid.allBlocks.at(i).y*scale);
            cv::Point br = cv::Point(tl.x + scale,tl.y+scale);

            cv::rectangle(drawingImage,tl,br,Scalar(0,0,255),-1);
        }
        if(m_grid.allBlocks.at(i).verticalOnly){
            cv::Point tl = cv::Point(m_grid.allBlocks.at(i).x*scale,m_grid.allBlocks.at(i).y*scale);
            cv::Point br = cv::Point(tl.x + scale,tl.y+scale);

            cv::rectangle(drawingImage,tl,br,Scalar(255,0,0),-1);
        }

        if(m_grid.allBlocks.at(i).closed){
            cv::Point tl = cv::Point(m_grid.allBlocks.at(i).x*scale,m_grid.allBlocks.at(i).y*scale);
            cv::Point br = cv::Point(tl.x + scale,tl.y+scale);

            //cv::rectangle(drawingImage,tl,br,Scalar(0,11,150),-1);
        }

    }
    cv::Point tl = cv::Point(0,0);
    cv::Point br = cv::Point(m_grid.m_cols*scale,m_grid.m_rows*scale);
    cv::rectangle(drawingImage,tl,br,Scalar(0,0,255),3);


}








void createGrid() {
	m_grid = grid(fieldMap->cols,fieldMap->rows);
}
vector<double> posToBlock(double x, double y, double fieldSize){
	//calculate the positions as percentages of the total field size
	double x_percent = x/fieldSize;
	//the top left of our map is our origin, however, our input assumes
	//that the origin is the bottom left. Therefore, we must invert the y
	double y_percent = 1-(y/fieldSize);
	//scale the percentages to the size of our map
	x_percent *= fieldMap->cols -1;
	y_percent *= fieldMap->rows -1;

	//cast the scaled positions to the nearest block because our
	//pathfinder can't compute fractions of blocks
	vector<double> finalBlockPos;
	finalBlockPos.push_back(round(x_percent));
	finalBlockPos.push_back(round(y_percent));
	finalBlockPos.push_back(x_percent);
	finalBlockPos.push_back(y_percent);
	return finalBlockPos;

}
bool scalar_equals(Scalar a, Scalar b){
	if(a.val[0] == b.val[0] && a.val[1]==b.val[1] && a.val[2]==b.val[2]){
		return true;
	}else{
		return false;
	}
}

bool calcPath(double my_X, double my_Y, double target_X, double target_Y, double fieldSize){



    //convert our pos to path-finding coordinates
	vector<double> my_pos_block = posToBlock(my_X,my_Y,fieldSize);
	vector<double> target_pos_block = posToBlock(target_X, target_Y,fieldSize);


    cv::Point myPosPoint = cv::Point((int) my_pos_block.at(0),(int) my_pos_block.at(1));
    cv::Point targetPosPoint = cv::Point((int) target_pos_block.at(0),(int) target_pos_block.at(1));


    if(targetPosPoint.x > m_grid.m_cols || targetPosPoint.x < 0
            || targetPosPoint.y > m_grid.m_rows || targetPosPoint.y < 0){
        return false;
    }


    m_grid.setTarget(targetPosPoint.x,targetPosPoint.y);

    __android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "f1");



    //loop through all of this pixels which are coded with different colors that apply different
	//constraints
	//for example, obstacles are coded by the color white
    int obstaclesDebug = 0;
	for(int r = 0; r < fieldMap->cols;r++){
		for(int c = 0; c < fieldMap->rows;c++){
			Scalar thisPixel = fieldMap->at<Vec3b>(c,r);

			if(scalar_equals(thisPixel,color_obstacle)){
				m_grid.setObstacle(r,c);
				obstaclesDebug++;
			}
			if(scalar_equals(thisPixel,color_allowsOnlyHorizontal)){
				m_grid.setOnlyHorizontal(r,c);
			}
			if(scalar_equals(thisPixel,color_allowsOnlyVertical)){
				m_grid.setOnlyVertical(r,c);
			}
			if(scalar_equals(thisPixel,color_penalty1)){
				m_grid.getBlock(r,c)->g_score = 1.5;
			}
		}
	}
	//if we are seated on an invalid block, abort
	if(m_grid.getBlock(myPosPoint.x,myPosPoint.y)->invalid){
	    return false;
	}
    __android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "f2");









    //we want to remember if we changed the target pos so that we can re-add the actual
    //target pos to the m_filteredsteps later.
    bool changedTarget = false;
    cv::Point originalTargetPos = targetPosPoint;

	////////////////////////If our target pos is invalid///////////////////
	if(m_grid.getBlock(targetPosPoint.x,targetPosPoint.y)->invalid){
        changedTarget = true;
		//1. find the angle of the target relative to the center using Math.atan2
		Point centerPosition = Point(m_grid.m_cols/2,m_grid.m_rows/2);
		double deltaY = targetPosPoint.y-centerPosition.y;
		double deltaX = targetPosPoint.x-centerPosition.x;
		double angleFromCenter = atan2(deltaY,deltaX);
        //2. Find the distance of the target relative to the center using the
		//distance formula/pythagorean theorem
		double distFromCenter = sqrt(pow(deltaX,2)+pow(deltaY,2));

		//3. increment the distance from the center until our position is
		//no-longer on an invalid block. This requires recalculating deltas
		for(;;) {
			//increment the distance from the center
			distFromCenter += 1;
			//calculate our deltas using trigonometry
			deltaY = sin(angleFromCenter)*distFromCenter;
			deltaX = cos(angleFromCenter)*distFromCenter;
			//calculate our absolute coordinates
			double x_abs = deltaX + centerPosition.x;
			double y_abs = deltaY + centerPosition.y;
			//round our absolute coordinates to the nearest block
			int x_block = round(x_abs);
			int y_block = round(y_abs);
			//if the new, moved target position is no-longer on an invalid block,
			//set the target to the new position, and break the loop
			if(!m_grid.getBlock(x_block,y_block)->invalid){
                m_grid.setTarget(x_block,y_block);
                break;
			}
		}
	}



    __android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "f3");








    ////////////////////////////COMMIT TO A POSITION AROUND OBSTICALS////////////////////////////
	/// 1. Draw a line from our position to the target position
    int obstaclesBefore = countObsticals(fieldMap);
    Mat collisionLineMat = fieldMap->clone();
    cv::line(collisionLineMat,myPosPoint,targetPosPoint,Scalar(0,0,0));
    int obstacleCountAfter = countObsticals(&collisionLineMat);
    collisionLineMat.release();
    /// 2. If that line obstructs any obsticals AND the last calculation did,
    /// 3.      -> return false
    /// 4. Else
    /// 5.      -> happy day!
    if(obstacleCountAfter < obstaclesBefore){
        if(obstructedObstaclesLast){
            return false;
        }
        obstructedObstaclesLast = true;
    }else{
        obstructedObstaclesLast = false;
    }


    __android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "f4");

    m_grid.findPath((int) my_pos_block.at(0),(int) my_pos_block.at(1));

    __android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "f5");

	my_X = (int) my_pos_block.at(0);
	my_Y = (int) my_pos_block.at(1);
	target_X = (int) target_pos_block.at(0);
	target_Y = (int) target_pos_block.at(1);

	std::vector<stepPoint> pointSteps =  m_grid.m_steps;
    std::vector<cv::Point2f> filteredSteps = vector<cv::Point2f>();

	//start at the first point, since it is in reverse order
	stepPoint currentPoint = pointSteps.at(0);
	bool finished = false;


	int currentMandatoryPointIndex = pointSteps.size()+10;
	for(int i =0;i<pointSteps.size();i++){
		if(pointSteps.at(i).mandatory){
			currentMandatoryPointIndex = i;
			break;
		}
	}

	for(;;){
		//go in backwards order through the ArrayList of points
		for(int i = pointSteps.size()-1; i>=0;i--){
			int obsticalCountBefore = countObsticals(fieldMap);
			Mat collisionLineMat = fieldMap->clone();
			//draw a line from the current point to the finish
			//if it cuts through any obsticals, there will be less
			//obstical pixels after drawing a line from start to finish
			cv::line(collisionLineMat,currentPoint.p,pointSteps.at(i).p,Scalar(0,0,0));
			int obsticalCountAfter = countObsticals(&collisionLineMat);
			collisionLineMat.release();

			if(obsticalCountBefore == obsticalCountAfter && i <= currentMandatoryPointIndex){
				if(i==currentMandatoryPointIndex){
					currentMandatoryPointIndex = pointSteps.size()+10;
					for(int j =i+1;j<pointSteps.size();j++){
						////////possibly i pointsteps.adj doesn't equal null
						if(pointSteps.at(j).mandatory){
							currentMandatoryPointIndex = j;
						}
					}
				}


				//if we can get to this point in a straight line
				//without going through any obsticals, add it to the step list
				currentPoint = pointSteps.at(i);
				filteredSteps.push_back(pointSteps.at(i).p);
				//if we have gotten to the finish line
				if(i == pointSteps.size()-1){
					finished = true;
				}
				break;
			}
		}
		if(finished){break;}
	}


    //if we moved the target pos out from the center, we need
    //to re-add the original target pos.
    if(changedTarget){
        filteredSteps.push_back(originalTargetPos);
    }

	m_filteredSteps = filteredSteps;




    ///////////////////////////Convert Filtered Steps to Field Positions//////////////
    for(int i = 0; i<m_filteredSteps.size();i++){
         //m_filteredSteps.at(i).x = m_filteredSteps.at(i).x/m_grid.m_cols;

        m_filteredSteps.at(i).x = (m_filteredSteps.at(i).x/(m_grid.m_cols-1))*fieldSize;
        //convert the y to a percentage and then invert it before scaling it to the fieldSize
        //m_filteredSteps.at(i).y = m_filteredSteps.at(i).y/m_grid.m_rows;
       // __android_log_print(ANDROID_LOG_DEBUG, "LOG_TAG", "x: %f y: %f" ,m_filteredSteps.at(i).x,m_filteredSteps.at(i).y);

        m_filteredSteps.at(i).y = (1-(m_filteredSteps.at(i).y/(m_grid.m_rows-1)))*fieldSize;
    }


	return true;
}

int countObsticals(Mat* map){
	int obsticals = 0;
	for(int r = 0; r < map->cols;r++){
		for(int c = 0; c < map->rows;c++){
			Scalar thisPixel = map->at<Vec3b>(c,r);
			if(scalar_equals(thisPixel,color_obstacle)){
				obsticals++;
			}
		}
	}
	return obsticals;
}