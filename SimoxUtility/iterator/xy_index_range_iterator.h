#pragma once

#include <iterator>

namespace simox::iterator
{


    struct Point
    {
        int x;
        int y;

        Point(int x, int y) : x(x), y(y)
        {
        }

        Point() = default;

        bool
        operator==(const Point& rhs) const
        {
            return x == rhs.x && y == rhs.y;
        }

        static Point
        zero()
        {
            return {0, 0};
        }
    };

    /**
* The XYIndexRangeIterator class.
*
* The aim of this class is to provide a convenient way to iterate over 2D arrays
* This class provides a 2D iterator that returns all indices to the 2D array, e.g.
*
*   My2DArray arr(10, 20);
*   for(auto [x,y] : XYIndexRangeIterator(arr)){
*      auto val = arr.at(x,y);
*      // do the magic
*   }
*
* This is hightly inspired by cartographer::mapping::XYIndexRangeIterator
* see: https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/2d/xy_index.h
*/
    template <typename IndexType = Point>
    class XYIndexRangeIterator : public std::iterator<std::input_iterator_tag, IndexType>
    {
    public:
        // Constructs a new iterator for the specified range.
        XYIndexRangeIterator(const IndexType& min_xy_index, const IndexType& max_xy_index) :
            min_xy_index(min_xy_index), max_xy_index(max_xy_index), xy_index(min_xy_index)
        {
        }

        // Constructs a new iterator for everything contained in 'cell_limits'.
        explicit XYIndexRangeIterator(const IndexType& cell_limits) :
            XYIndexRangeIterator(IndexType::zero(), IndexType(cell_limits.x - 1, cell_limits.y - 1))
        {
        }

        XYIndexRangeIterator&
        operator++()
        {
            if (xy_index.x < max_xy_index.x)
            {
                ++xy_index.x;
            }
            else
            {
                xy_index.x = min_xy_index.x;
                ++xy_index.y;
            }
            return *this;
        }

        IndexType&
        operator*()
        {
            return xy_index;
        }

        bool
        operator==(const XYIndexRangeIterator& other) const
        {
            return xy_index == other.xy_index;
        }

        bool
        operator!=(const XYIndexRangeIterator& other) const
        {
            return !operator==(other);
        }

        XYIndexRangeIterator
        begin() const
        {
            return XYIndexRangeIterator(min_xy_index, max_xy_index);
        }

        XYIndexRangeIterator
        end() const
        {
            XYIndexRangeIterator it = begin();
            it.xy_index = IndexType(min_xy_index.x, max_xy_index.y + 1);
            return it;
        }

    private:
        IndexType min_xy_index;
        IndexType max_xy_index;
        IndexType xy_index;
    };


} // namespace simox::iterator
