#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"
#include "constants.h"

struct COORDINATES {
    float start_x;
    float start_y;
    float end_x;
    float end_y;
};

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path) {
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is)
        return std::nullopt;

    auto size = is.tellg();
    std::vector<std::byte> contents(size);

    is.seekg(0);
    is.read((char *) contents.data(), size);

    if (contents.empty())
        return std::nullopt;
    return std::move(contents);
}

static float GuardedCoordinateInput(const std::string &input_message){
    float input;

    std::cout << input_message;
    std::cin >> input;

    if(input < LOWER_BOUNDARY){
        std::cout << "Wrong input " << input << ". Please provide an input value greater than " << LOWER_BOUNDARY << "\n";
        input = GuardedCoordinateInput(input_message);
    }

    if(input > UPPER_BOUNDARY){
        std::cout << "Wrong input " << input << ". Please provide an input value lower than " << UPPER_BOUNDARY << "\n";
        input = GuardedCoordinateInput(input_message);
    }

    return input;
}

static COORDINATES ReadInCoordinatesFromCin() {
    struct COORDINATES coordinates{};

    coordinates.start_x = GuardedCoordinateInput("Please enter a start x value: ");
    coordinates.start_y = GuardedCoordinateInput("Please enter a start y value: ");
    coordinates.end_x = GuardedCoordinateInput("Please enter a end x value: ");
    coordinates.end_y = GuardedCoordinateInput("Please enter a end y value: ");

    return coordinates;
}

int main(int argc, const char **argv) {
    std::string osm_data_file;
    if (argc > 1) {
        for (int i = 1; i < argc; ++i)
            if (std::string_view{argv[i]} == "-f" && ++i < argc)
                osm_data_file = argv[i];
    } else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }

    std::vector<std::byte> osm_data;

    if (osm_data.empty() && !osm_data_file.empty()) {
        std::cout << "Reading OpenStreetMap data from the following file: " << osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if (!data)
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    struct COORDINATES coordinates = ReadInCoordinatesFromCin();

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{
                                    model,
                                    coordinates.start_x,
                                    coordinates.start_y,
                                    coordinates.end_x,
                                    coordinates.end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed,
                                        30};
    display.size_change_callback([](io2d::output_surface &surface) {
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface &surface) {
        render.Display(surface);
    });
    display.begin_show();
}
