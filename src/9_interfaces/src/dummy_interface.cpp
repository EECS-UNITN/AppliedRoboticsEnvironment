#include "student_interface.hpp"

#include <stdexcept>
#include <sstream>
namespace student {

  std::string initImageFolder(){
  	return "/home/valerio/Pictures/";
  }	

  void genericImageListener(const cv::Mat& img_in, std::string topic){
  	static size_t id = 0;
  	static bool init = false;
  	static std::string folder_path;

  	if(!init){
  		folder_path = initImageFolder();
  		init = true;
  	}
    	    
    cv::imshow( topic, img_in);
    char c;
    c = cv::waitKey(30);
    
    std::stringstream file;
    switch (c) {    	
		case 's':		
			file << folder_path << std::setfill('0') 
					<< std::setw(2)  << (id++) << ".jpg";
		 	cv::imwrite( file.str(), img_in );
		 	std::cout << "Saved image " << file.str() << std::endl;
		 	break;
		default:
				break;
    }
	
  	
    //throw std::logic_error( "STUDENT FUNCTION NOT LOADED" );
  }

	void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
					const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs){

    throw std::logic_error( "STUDENT FUNCTION NOT LOADED" );	}
}
