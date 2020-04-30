#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace::std;

using std::cin;
using std::cout;
using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

bool CheckMinMax(float num, float min, float max) {
    return (num >= min && num <= max);
}

bool CheckValidInputs(float& i1, float& i2, float& i3, float& i4) {
    return (CheckMinMax(i1, 0, 100) && CheckMinMax(i2, 0, 100) && CheckMinMax(i3, 0, 100) && CheckMinMax(i4, 0, 100));
}

void GetUserInput(float& start_x, float& start_y, float& end_x, float& end_y) {
    // get user input for starting and goal coordinates
    while (1) {
        cout << "Enter x coordinate for start point (0..100). Confirm with Enter:";
        cin >> start_x;
        cout << "Enter y coordinate for start point (0..100). Confirm with Enter:";
        cin >> start_y;
        cout << "Enter x coordinate for goal point (0..100). Confirm with Enter:";
        cin >> end_x;
        cout << "Enter y coordinate for goal point (0..100). Confirm with Enter:";
        cin >> end_y;
        cout << "Goal x: " << end_x << " y: " << end_y << "\n";
        if (CheckValidInputs(start_x, start_y, end_x, end_y))
            return;
        else
            cout << "It seems your values are out or range. \n";
    }
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x, start_y, end_x, end_y;
    GetUserInput(start_x, start_y, end_x, end_y);

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
