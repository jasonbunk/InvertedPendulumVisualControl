/*
 * Miscellaneous shared utility functions.
 *
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: Apache License Version 2.0, January 2004
 * Copyright (c) 2015 Jason Bunk
 */
#include "SUtils.h"
#include "OS_FolderBrowser_tinydir.h"
#include <fstream>



//returns the chars after the delim, maybe including the delim (determined by the boolean)
std::string trim_chars_after_delim(std::string & thestr, char delim, bool include_delim_in_returned_trimmed_end)
{
    size_t lastDelimPos = thestr.find_last_of(delim);
    if(lastDelimPos > 0 && lastDelimPos < thestr.size())
    {
        std::string returned_extension;
        if(include_delim_in_returned_trimmed_end)
            returned_extension = thestr.substr(lastDelimPos);
        else
            returned_extension = thestr.substr(lastDelimPos+1);
        thestr.erase(thestr.begin()+lastDelimPos, thestr.end());
        return returned_extension;
    }
    return "";
}

//returns the chars after the FIRST INSTANCE OF THE delim, maybe including the delim (determined by the boolean)
std::string trim_chars_after_first_instance_of_delim(std::string & thestr, char delim, bool include_delim_in_returned_trimmed_end)
{
	size_t lastDelimPos = thestr.find_first_of(delim);
	if(lastDelimPos > 0 && lastDelimPos < thestr.size()) {
		std::string returned_extension;
		if(include_delim_in_returned_trimmed_end)
			returned_extension = thestr.substr(lastDelimPos);
		else
			returned_extension = thestr.substr(lastDelimPos+1);
		thestr.erase(thestr.begin()+lastDelimPos, thestr.end());
		return returned_extension;
	}
	return "";
}


//returns what was before the delim
//leaves behind the chars after the FIRST INSTANCE OF THE delim, maybe including the delim (determined by the boolean)
std::string trim_chars_after_first_instance_of_delim_return_first(std::string & thestr, char delim, bool include_delim_in_returned_trimmed_end)
{
	size_t lastDelimPos = thestr.find_first_of(delim);
	if(lastDelimPos > 0 && lastDelimPos < thestr.size()) {
		std::string returned_before_delim;
		if(include_delim_in_returned_trimmed_end)
			returned_before_delim = thestr.substr(0, lastDelimPos-1);
		else
			returned_before_delim = thestr.substr(0, lastDelimPos);
		thestr.erase(thestr.begin(), thestr.begin()+lastDelimPos+1);
		return returned_before_delim;
	}
	std::string returnval(thestr);
	thestr.clear();
	return returnval;
}


std::string get_extension_from_filename(const std::string & filename)
{
    size_t lastPeriodPos = filename.find_last_of('.'); //INCLUDING the period

    if(lastPeriodPos > 0 && lastPeriodPos < filename.size())
    {
        return filename.substr(lastPeriodPos);
    }

    return "";
}


std::string eliminate_extension_from_filename(std::string & filename)
{
    return trim_chars_after_delim(filename, '.', true);
}


std::vector<std::string> GetFilenamesOfTypeInFolder(std::string folder_dir_name, std::string extension_of_file)
{
	std::vector<std::string> returnedFnames;
	tinydir_dir dir;
	tinydir_open(&dir, folder_dir_name.c_str());
	while(dir.has_next) {
		tinydir_file file;
		tinydir_readfile(&dir, &file);
		if(file.is_dir == false && file.name[0] != '.') {
			if(!__stricmp(get_extension_from_filename(file.name).c_str(),extension_of_file.c_str())) {
				returnedFnames.push_back(file.name);
			}
		}
		tinydir_next(&dir);
	}
	tinydir_close(&dir);
	return returnedFnames;
}





bool check_if_file_exists(const std::string & filename)
{
	std::ifstream myfile(filename);
	if(myfile.is_open() && myfile.good()) {
		myfile.close();
		return true;
	}
	return false;
}
bool check_if_directory_exists(const std::string & dir_name)
{
	return dir_name.empty()==false && check_if_file_exists(dir_name);
}
int CountFilesOfTypeInFolder(std::string folder_dir_name, std::string desired_extension)
{
	int returned_num_images = 0;
	tinydir_dir dir;
	tinydir_open(&dir, folder_dir_name.c_str());
	while(dir.has_next) {
		tinydir_file file;
		tinydir_readfile(&dir, &file);
		if(file.is_dir == false && file.name[0] != '.' && get_extension_from_filename(file.name)==desired_extension) {
			returned_num_images++;
		}
		tinydir_next(&dir);
	}
	tinydir_close(&dir);
	return returned_num_images;
}




