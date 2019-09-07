#pragma once

#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

class FSHelper{

public:

    /**
     * @brief filterFilesByRegx     根据正则表达式过滤文件
     * @param paths[in]     表明哪些目录下的文件需要被过滤
     * @param files[out]    过滤后的文件列表
     * @param regx          正则表达式
     */
    static void filterFilesByRegx(const std::vector<std::string> &paths, std::vector<std::string> &files,\
                                  std::string regx) __attribute__((weak)){

        /**
         * @brief reg   正则表达式
         */
        boost::xpressive::cregex reg = boost::xpressive::cregex::compile(regx);

        BOOST_FOREACH(std::string path, paths){

            boost::filesystem::path libDir(path);
            if (! boost::filesystem::exists(libDir)) {
                    continue;
            }

            boost::filesystem::directory_iterator end;

            /**
         * 遍历目录，获取目录下的所有文件
         */
            for(boost::filesystem::directory_iterator pos(libDir); pos != end; ++pos){
                if(boost::filesystem::is_directory(*pos)){
                    continue;
                }

                /**
             * @brief what  临时保存正则表达式的查找结果
             */
                boost::xpressive::cmatch what;

                /**
             * 通过正则表达式 获取文件名中的检测器名称
             */
                std::string tmp(pos->path().filename().string());

                /**
                 *  如果直接使用pos->path().filename().string().c_str()作为第一参数的话，那么是可悲的。
                 *  因为还有调用regex_search这个函数的时候，这个c_str所指向的char *里的数据就已经没了。
                 *  因为c_str返回的数据是由string维护的
                 */
                if(boost::xpressive::regex_search(tmp.c_str(), what, reg)){
                    /**
                 * 这里我们假定匹配到的第一个字符串就是识别器的名称
                 */
                    files.push_back(what[0]);
                }
            }
        }

    }

};
