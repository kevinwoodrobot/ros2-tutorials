#include <polygon_base/regular_polygon.hpp>
#include <cmath>

namespace polygon_plugins
{
    class Square : public polygon_base::RegularPolygon
    {
        public:
            void initialize(double side_length) override
            {
                side_length_ = side_length;
            }

            double area() override
            {
                return side_length_ * side_length_;
            }

        protected:
            double side_length_;
    };

    class Triangle : public polygon_base::RegularPolygon
    {
    public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return 0.5 * side_length_ * getHeight();
        }

        double getHeight()
        {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
        }

    protected:
        double side_length_;
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)