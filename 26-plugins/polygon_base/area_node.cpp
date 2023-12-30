#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char** argv)
{
    // To avoid unused parameter warnings
    (void) argc;
    (void) argv;

    pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

    try
    {
        std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
        triangle->initialize(10.0);

        std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
        square->initialize(10.0);

        printf("Triangle area: %.2f\n", triangle->area());
        printf("Square area: %.2f\n", square->area());
    }
    catch(pluginlib::PluginlibException& ex)
    {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    return 0;
}