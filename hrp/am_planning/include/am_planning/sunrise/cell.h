#ifndef _CELL_H_
#define _CELL_H_

#include <stdint.h>
#include <iostream>

enum Cell_type
{
    FREE,
    OBSTACLE,
    BOUNDARY_WIRE,
    MAPPED_BOUNDARY,
    VIRTUAL_BOUNDARY,
    GNSS_SHADOW,
    CHARGING_STATION
};

class Cell
{

protected:
    int32_t east;
    int32_t north;
    int32_t priority;
    Cell_type type;
    bool covered;
    bool clearable;

    int32_t get_new_priority() const;

public:
    Cell(int32_t east, int32_t north);
    Cell(int32_t east, int32_t north, Cell_type cell_type);
    Cell(int32_t east, int32_t north, bool covered);
    Cell(int32_t east, int32_t north, Cell_type cell_type, bool covered);
    virtual ~Cell();

    // inline bool operator==(const Cell& rhs) const
    // {
    //     return (this->get_north() == rhs.get_north() && this->get_east() == rhs.get_east());
    // }

    int32_t get_north() const
    {
        return north;
    }
    int32_t get_east() const
    {
        return east;
    }
    int32_t get_priority() const
    {
        return priority;
    }
    Cell_type get_type() const
    {
        return type;
    }
    bool is_covered() const
    {
        return covered;
    }
    bool is_clearable() const
    {
        return clearable;
    }
    void set_type(Cell_type new_type)
    {
        type = new_type;
    }
    void set_covered(const bool value)
    {
        covered = value;
    }
    void set_clearable(const bool value)
    {
        clearable = value;
    }
    void update_priority();
};

bool operator==(const Cell& cell1, const Cell& cell2);
bool operator!=(const Cell& cell1, const Cell& cell2);
std::ostream& operator<<(std::ostream& os, const Cell& obj);

#endif // _CELL_H_
