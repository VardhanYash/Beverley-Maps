/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "altitude_data.h"

std::string elevation_info;

static size_t write_data(void *buffer, size_t size, size_t nmemb, void *userp) {
    size = size;
    
    if (buffer && nmemb && userp) {
        MyCustomStruct *pMyStruct = (MyCustomStruct *)userp;

        // Writes to struct passed in from main
        if (pMyStruct->response == nullptr) {
            // Case when first time write_data() is invoked
            pMyStruct->response = new char[nmemb + 1];
            strncpy(pMyStruct->response, (char *)buffer, nmemb);
        }
        else {
            // Case when second or subsequent time write_data() is invoked
            char *oldResp = pMyStruct->response;

            pMyStruct->response = new char[pMyStruct->size + nmemb + 1];

            // Copy old data
            strncpy(pMyStruct->response, oldResp, pMyStruct->size);

            // Append new data
            strncpy(pMyStruct->response + pMyStruct->size, (char *)buffer, nmemb);

            delete []oldResp;
        }

        pMyStruct->size += nmemb;
        pMyStruct->response[pMyStruct->size] = '\0';
    }

    return nmemb;
}
 
void write_altitude_data(boost::property_tree::ptree &ptRoot) {
        
    
    BOOST_FOREACH(boost::property_tree::ptree::value_type &featVal, ptRoot.get_child("data")){
        double elevation; 
        std::string height;  
        
        std::stringstream parse(featVal.second.data());
        parse >> elevation;
        height = std::to_string((int) elevation);
        std::string alt_text;
            
        if(elevation > 260){
            alt_text = "This intersection is at a high altitude of ";
        }
            
        else if(elevation > 110){
            alt_text = "This intersection is at a moderate altitude of ";
        }
            
        else {
            alt_text = "This intersection is at a low altitude of ";
        }
                
        elevation_info = alt_text + height + " meters";
  
    }
        
    return;
}


void altitude_api_set_up(){
    
    CURLcode res = curl_global_init(CURL_GLOBAL_ALL);
    if (res != CURLE_OK) {
        std::cout << "ERROR: Unable to initialize libcurl" << std::endl;
        std::cout << curl_easy_strerror(res) << std::endl;
        return;
    }

    CURL *curlHandle = curl_easy_init();
    if ( !curlHandle ) {
        std::cout << "ERROR: Unable to get easy handle" << std::endl;
        return;
    } 
    else {
        char errbuf[CURL_ERROR_SIZE] = {0};
        MyCustomStruct myStruct;
        
        int size = URL_altitude.size() + 1;
        
        char *targetURL = new char[size];
        
        std::strcpy(targetURL,URL_altitude.c_str());
        

        res = curl_easy_setopt(curlHandle, CURLOPT_URL, targetURL);
        if (res == CURLE_OK)
            res = curl_easy_setopt(curlHandle, CURLOPT_ERRORBUFFER, errbuf);
        if (res == CURLE_OK)
            res = curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, write_data);
        if (res == CURLE_OK)
            res = curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, &myStruct);

        myStruct.url = targetURL;

        if (res != CURLE_OK) {
            std::cout << "ERROR: Unable to set libcurl option" << std::endl;
            std::cout << curl_easy_strerror(res) << std::endl;
        } else {
            res = curl_easy_perform(curlHandle);
        }

        if (res == CURLE_OK) {
            // Create an empty proper tree
            boost::property_tree::ptree ptRoot;

            /* Store JSON data into a Property Tree
             *
             * read_json() expects the first parameter to be an istream object,
             * or derived from istream (e.g. ifstream, istringstream, etc.).
             * The second parameter is an empty property tree.
             *
             * If your JSON data is in C-string or C++ string object, you can
             * pass it to the constructor of an istringstream object.
             */
            std::istringstream issJsonData(myStruct.response);
            read_json(issJsonData, ptRoot);
            
            // Parsing and printing the data
            try {
                write_altitude_data(ptRoot);
            } catch (const char *errMsg) {
                std::cout << "ERROR: Unable to fully parse the Weather JSON data" << std::endl;
                std::cout << "Thrown message: " << errMsg << std::endl;
            }
        } else {
            std::cout << "ERROR: res == " << res << std::endl;
            std::cout << errbuf << std::endl;
        }

        if (myStruct.response)
            delete []myStruct.response;

        curl_easy_cleanup(curlHandle);
        curlHandle = nullptr;
        
        delete [] targetURL;
    }

    curl_global_cleanup();

    return;
}