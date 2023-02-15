#pragma once

namespace ras_data_classes
{
    /**
     * @brief Struct that acts as a container for storing the call with the according f_cost value
     * Contains overwriten > and < operators for multiset comparator functions
     *
     */
    struct Cell
    {
        int cell_index;
        float f_cost;

        bool operator<(const Cell& rhs) const
        {
            return this->f_cost < rhs.f_cost;
        }

        bool operator>(const Cell& rhs) const
        {
            return this->f_cost > rhs.f_cost;
        }
    };

    enum NeighborType
    {
        FourWay = 4,
        EightWay = 8
    };
}