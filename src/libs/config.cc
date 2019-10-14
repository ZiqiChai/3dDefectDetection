#include "../../include/config.h"

void Config::setParameterFile(const std::string& fileName)
{
	if (config_ == NULL)
		config_ = new Config; //std::shared_ptr<Config>(new Config);
	config_->file_ = cv::FileStorage(fileName.c_str(), cv::FileStorage::READ);
	if (!config_->file_.isOpened()){
		std::cerr << "parameter file" << fileName << "does not exist." << std::endl;
		config_->file_.release();
		return;
	}
}

Config::~Config()
{
	if (file_.isOpened())
		file_.release();
	delete config_;
}

Config* Config::config_ = NULL;