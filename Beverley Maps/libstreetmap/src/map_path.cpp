/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "map_path.h"

//The map_path for every location
std::string default_return = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";
std::string beijing = "/cad2/ece297s/public/maps/beijing_china.streets.bin";
std::string cairo = "/cad2/ece297s/public/maps/cairo_egypt.streets.bin";
std::string cape_town = "/cad2/ece297s/public/maps/cape-town_south-africa.streets.bin";
std::string golden_horseshoe = "/cad2/ece297s/public/maps/golden-horseshoe_canada.streets.bin";
std::string hamilton = "/cad2/ece297s/public/maps/hamilton_canada.streets.bin";
std::string hong_kong = "/cad2/ece297s/public/maps/hong-kong_china.streets.bin";
std::string iceland = "/cad2/ece297s/public/maps/iceland.streets.bin";
std::string interlaken = "/cad2/ece297s/public/maps/interlaken_switzerland.streets.bin";
std::string london = "/cad2/ece297s/public/maps/london_england.streets.bin";
std::string moscow = "/cad2/ece297s/public/maps/moscow_russia.streets.bin";
std::string new_delhi = "/cad2/ece297s/public/maps/new-delhi_india.streets.bin";
std::string new_york = "/cad2/ece297s/public/maps/new-york_usa.streets.bin";
std::string rio_de_janeiro = "/cad2/ece297s/public/maps/rio-de-janeiro_brazil.streets.bin";
std::string saint_helena = "/cad2/ece297s/public/maps/saint-helena.streets.bin";
std::string singapore = "/cad2/ece297s/public/maps/singapore.streets.bin";
std::string sydney = "/cad2/ece297s/public/maps/sydney_australia.streets.bin";
std::string tehran = "/cad2/ece297s/public/maps/tehran_iran.streets.bin";
std::string tokyo = "/cad2/ece297s/public/maps/tokyo_japan.streets.bin";
std::string toronto = "/cad2/ece297s/public/maps/toronto_canada.streets.bin";

std::string path(std::string location){
    //gets rid of all spaces and makes the user input all lowercase
    boost::to_lower(location);
    location.erase(remove(location.begin(), location.end(), ' '), location.end());
    
    
    //checks which map to load based on the user scanned input
    if(boost::algorithm::contains(location, "beijing")) {
        return beijing;
    }

    else if (boost::algorithm::contains(location, "cairo")) {
        return cairo;
    }

    else if (boost::algorithm::contains(location, "cape")) {
        return cape_town;
    }

    else if (boost::algorithm::contains(location, "horse")) {
        return golden_horseshoe;
    }

    else if (boost::algorithm::contains(location, "hamilton")) {
        return hamilton;
    }

    else if (boost::algorithm::contains(location, "hong")) {
        return hong_kong;
    }

    else if (boost::algorithm::contains(location, "iceland")) {
        return iceland;
    }

    else if (boost::algorithm::contains(location, "inter")) {
        return interlaken;
    }

    else if (boost::algorithm::contains(location, "london")) {
        return london;
    }

    else if(boost::algorithm::contains(location, "moscow")) {
        return moscow;
    }

    else if (boost::algorithm::contains(location, "delhi")) {
        return new_delhi;
    }

    else if(boost::algorithm::contains(location, "york")) {
        return new_york;
    }

    else if (boost::algorithm::contains(location, "rio")) {
        return rio_de_janeiro;
    }

    else if (boost::algorithm::contains(location, "saint")) {
        return saint_helena;
    }

    else if (boost::algorithm::contains(location, "singapore")) {
        return singapore;
    }

    else if (boost::algorithm::contains(location, "sydney")) {
        return sydney;
    }

    else if (boost::algorithm::contains(location, "tehran")) {
        return tehran;

    }

    else if (boost::algorithm::contains(location, "tokyo")) {
        return tokyo;
    }

    else if (boost::algorithm::contains(location, "toronto")) {
        return toronto;
    }

    else {
        std::cout << "Invalid map name entered, loading Toronto" << std::endl;
        return default_return;
    } 
}