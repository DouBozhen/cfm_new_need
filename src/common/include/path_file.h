#ifndef PATH_FILE_H
#define PATH_FILE_H

//#define ROS_LAUNCH

#ifdef ROS_LAUNCH

#define JOINT_PATH_FILE_DIR    "../force_master_ws/docs/path_files/joint_path.txt"
#define JOINT_IMPENDANCE_PATH_FILE_DIR    "../force_master_ws/docs/path_files/joint_path_impendance.txt"
#define CART_PATH_FILE_DIR    "../force_master_ws/docs/path_files/cart_path.txt"
#define CART_IMPENDANCE_PATH_FILE_DIR    "../force_master_ws/docs/path_files/cart_path_impendance.txt"

#define TRAINGING_JOINT_PATH_PROXIMAL_DIR    "../force_master_ws/docs/training_path_files/proximal/joint_path.txt"
#define TRAINGING_JOINT_PATH_IMPENDANCE_PROXIMAL_DIR    "../force_master_ws/docs/training_path_files/proximal/joint_path_impendance.txt"
#define TRAINGING_CART_PATH_PROXIMAL_DIR    "../force_master_ws/docs/training_path_files/proximal/cart_path.txt"
#define TRAINGING_CART_PATH_IMPENDANCE_PROXIMAL_DIR    "../force_master_ws/docs/training_path_files/proximal/cart_path_impendance.txt"

#define TRAINGING_JOINT_PATH_DISTAL_DIR    "../force_master_ws/docs/training_path_files/distal/joint_path.txt"
#define TRAINGING_JOINT_PATH_IMPENDANCE_DISTAL_DIR    "../force_master_ws/docs/training_path_files/distal/joint_path_impendance.txt"
#define TRAINGING_CART_PATH_DISTAL_DIR    "../force_master_ws/docs/training_path_files/distal/cart_path.txt"
#define TRAINGING_CART_PATH_IMPENDANCE_DISTAL_DIR    "../force_master_ws/docs/training_path_files/distal/cart_path_impendance.txt"

#define CONFIG_FILE_DIR    "../force_master_ws/docs/config.txt"
#define TIMEOUT_FILE_DIR    "../force_master_ws/docs/time_out.txt"
#define PUZZEL_GAME_SPEED_LIST_DIR    "../force_master_ws/docs/speed_list.csv"
#define PUZZEL_SCORE_DIR    "../force_master_ws/docs/puzzle_score.txt"

#else

#define JOINT_PATH_FILE_DIR    "../../../docs/path_files/joint_path.txt"
#define JOINT_IMPENDANCE_PATH_FILE_DIR    "../../../docs/path_files/joint_path_impendance.txt"
#define CART_PATH_FILE_DIR    "../../../docs/path_files/cart_path.txt"
#define CART_IMPENDANCE_PATH_FILE_DIR    "../../../docs/path_files/cart_path_impendance.txt"
#define CONFIG_FILE_DIR    "../../../docs/config.txt"
#define TIMEOUT_FILE_DIR    "../../../docs/time_out.txt"
#define PUZZEL_GAME_SPEED_LIST_DIR    "../../../docs/speed_list.csv"
#define PUZZEL_SCORE_DIR    "../../../docs/puzzle_score.txt"

#define TRAINGING_JOINT_PATH_PROXIMAL_DIR    "../../../docs/training_path_files/proximal/joint_path.txt"
#define TRAINGING_JOINT_PATH_IMPENDANCE_PROXIMAL_DIR    "../../../docs/training_path_files/proximal/joint_path_impendance.txt"
#define TRAINGING_CART_PATH_PROXIMAL_DIR    "../../../docs/training_path_files/proximal/cart_path.txt"
#define TRAINGING_CART_PATH_IMPENDANCE_PROXIMAL_DIR    "../../../docs/training_path_files/proximal/cart_path_impendance.txt"

#define TRAINGING_JOINT_PATH_DISTAL_DIR    "../../../docs/training_path_files/distal/joint_path.txt"
#define TRAINGING_JOINT_PATH_IMPENDANCE_DISTAL_DIR    "../../../docs/training_path_files/distal/joint_path_impendance.txt"
#define TRAINGING_CART_PATH_DISTAL_DIR    "../../../docs/training_path_files/distal/cart_path.txt"
#define TRAINGING_CART_PATH_IMPENDANCE_DISTAL_DIR    "../../../docs/training_path_files/distal/cart_path_impendance.txt"


#endif

#endif