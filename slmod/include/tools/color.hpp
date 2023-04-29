#ifndef __THREAD_COLOR_HPP__
#define __THREAD_COLOR_HPP__

// #include <map>
#include <vector>
#include <map>

namespace Common_tools
{
    class COLOR {

        public:
        COLOR(){

            
            
            std::vector<int> rgb(3);
            int l = 1;
            for(int i = 0; i < 6; i++){

                // rgb.clear();
                rgb[0] = i * 42;
                for(int j = 0; j < 6; j++){

                    if(i % 2 == 0){

                        int m = 5 - j;
                        rgb[1] = m * 42;
                    }
                    else{
                        rgb[1] = j * 42;
                    }

                    

                    for(int k = 0; k < 6; k++){

                        rgb[2] = 255;
                        
                        color_pro.push_back(std::pair<int, std::vector<int>>(l, rgb));
                        l++;
                    }
                }
                
            }
            color_pro.resize(196);
        }


        std::vector<std::pair<int, std::vector<int>>> color_pro;
    };
}

#endif