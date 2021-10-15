#include <cstring>
#include <arpa/inet.h>
#include "arm_state.h"

void ArmState::decodeVarFromProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len, VarMessage &var_msg)
{
	var_msg.title_size = *(uint8_t *)&buf[offset];
	offset += sizeof(var_msg.title_size);

	memcpy(&var_msg.msg_title, &buf[offset], sizeof(char) * var_msg.title_size); 
	var_msg.msg_title[var_msg.title_size] = '\0';
	offset += var_msg.title_size;

	memcpy(&var_msg.msg_text, &buf[offset], len - offset);   
	var_msg.msg_text[len - offset] = '\0';
}

void ArmState::decodeGlobalParamsSetupFromProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len, GlobalParamsSetupMessage &global_setup)
{
	global_setup.start_index = *(uint16_t *)&buf[offset];
	global_setup.start_index = ntohs(global_setup.start_index);
	offset += sizeof(global_setup.start_index);

	memcpy(&global_setup.param_names, &buf[offset], len - offset);  
	global_setup.param_names[len - offset] = '\0';

    int index = 0;
    std::string global_name = global_setup_msg_.param_names;
    global_params_names_.push_back(global_name);

    for (int i = 1; i != sizeof(global_setup_msg_.param_names) - 2; ++i)
    {
        if (global_setup_msg_.param_names[i] == '\n')
        {
            index++;
            global_name = &global_setup_msg_.param_names[i+1];
            global_params_names_.push_back(global_name);
        }
    }
}

void ArmState::decodeGlobalParamsUpdataFromProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len, GlobalParamsUpdateMessage &global_update)
{
	global_update_msg_.start_index = buf[offset];
	offset += sizeof(global_update_msg_.start_index);

	while (offset < 1366 + len)
	{
		global_update_msg_.value_type = buf[offset];
		offset += sizeof(global_update_msg_.value_type);

 		switch ((int)global_update_msg_.value_type)
		{
            case 0:
                break;
            case 3:
            {
                global_update_msg_.string_len = *(int16_t *)&buf[offset];
                offset += sizeof(global_update_msg_.string_len);

                memcpy(&global_update_msg_.string_value, &buf[offset], sizeof(char) * global_update_msg_.string_len); 
                offset += global_update_msg_.string_len;

                global_update_msg_.string_value[global_update_msg_.string_len] = '\0';
            }
			    break;
            case 4:
            {
                global_update_msg_.var_string_len = *(int16_t *)&buf[offset];
                offset += sizeof(global_update_msg_.var_string_len);

                memcpy(&global_update_msg_.var_string_value, &buf[offset], sizeof(char)* global_update_msg_.var_string_len); 
                offset += global_update_msg_.var_string_len;

                global_update_msg_.var_string_value[global_update_msg_.var_string_len] = '\0';
            }
			    break;
            case 5:
            {
                global_update_msg_.list_length = *(int16_t *)&buf[offset];
                offset += sizeof(global_update_msg_.list_length);

                for (int i = 0; i < global_update_msg_.list_length; ++i)
                {
                    // global_update_msg_.list_val_type = buf[offset];
                    // offset += sizeof(global_update_msg_.list_val_type);
                }
            }
			    break;
            case 10:
            {
                uint32_t temp1 = *(uint32_t*)&buf[offset];
                global_update_msg_.pose.point.x = ntohf(temp1);
                offset += sizeof(temp1);

                temp1 = *(uint32_t*)&buf[offset];
                global_update_msg_.pose.point.y = ntohf(temp1);
                offset += sizeof(temp1);

                temp1 = *(uint32_t*)&buf[offset];
                global_update_msg_.pose.point.z = ntohf(temp1);
                offset += sizeof(temp1);

                temp1 = *(uint32_t*)&buf[offset];
                global_update_msg_.pose.rpy.rx = ntohf(temp1);
                offset += sizeof(temp1);

                temp1 = *(uint32_t*)&buf[offset];
                global_update_msg_.pose.rpy.ry = ntohf(temp1);
                offset += sizeof(temp1);

                temp1 = *(uint32_t*)&buf[offset];
                global_update_msg_.pose.rpy.rz = ntohf(temp1);
                offset += sizeof(temp1);
            }
                break;
            case 12:
            {
                uint8_t tmp = buf[offset];
                if (tmp > 0) global_update_msg_.bool_val = true;
                else global_update_msg_.bool_val = false;
                offset += sizeof(tmp);
            }
                break;
            case 13:
            {
                uint32_t temp3 = *(uint32_t*)&buf[offset];
                global_update_msg_.num_value = ntohf(temp3);
                offset += sizeof(temp3);
            }
                break;
            case 14:
            {	
                global_update_msg_.int_val = *(int32_t*)&buf[offset];
                global_update_msg_.int_val = ntohl(global_update_msg_.int_val);
                offset += sizeof(global_update_msg_.int_val);
            }
            break;
            case 15:
            {
                uint32_t temp2 = *(int32_t*)&buf[offset];
                offset += sizeof(temp2);
                global_update_msg_.float_val = ntohf(temp2);
            }
                break;
            default:
                break;
		}
		//offset += sizeof(global_update_msg_.value_type);
	}
}

