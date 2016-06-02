
#include "am_planning/sunrise/cell.h"

Cell::Cell(int32_t east, int32_t north)
    : east(east)
    , north(north)
    , priority(get_new_priority())
    , type(FREE)
    , covered(false)
    , clearable(true)
{
}

Cell::Cell(int32_t east, int32_t north, Cell_type cell_type)
    : east(east)
    , north(north)
    , priority(get_new_priority())
    , type(cell_type)
    , covered(false)
    , clearable(true)
{
}

Cell::Cell(int32_t east, int32_t north, bool covered)
    : east(east)
    , north(north)
    , priority(get_new_priority())
    , type(FREE)
    , covered(covered)
    , clearable(true)
{
}

Cell::Cell(int32_t east, int32_t north, Cell_type cell_type, bool covered)
    : east(east)
    , north(north)
    , priority(get_new_priority())
    , type(cell_type)
    , covered(covered)
    , clearable(true)
{
}

Cell::~Cell()
{
}

int32_t Cell::get_new_priority() const
{
    static int32_t latest_priority = 0;

    return latest_priority++;
}

void Cell::update_priority()
{
    priority = get_new_priority();
}

bool operator==(const Cell& cell1, const Cell& cell2)
{
    return (cell1.get_north() == cell2.get_north() && cell1.get_east() == cell2.get_east());
}

bool operator!=(const Cell& cell1, const Cell& cell2)
{
    return !(cell1 == cell2);
}

std::ostream& operator<<(std::ostream& os, const Cell& obj)
{
    os << "Cell at (east, north): (" << obj.get_east() << ", " << obj.get_north() << ")";

    return os;
}
