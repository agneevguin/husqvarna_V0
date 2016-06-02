#ifndef _CELL_COMPARATORS_H_
#define _CELL_COMPARATORS_H_

#include <iostream>
#include "cell.h"

class StaticCellComparator
{
public:
    bool operator()(const Cell& cell1, const Cell& cell2)
    {
        if (cell1.get_north() > cell2.get_north())
        {
            return true;
        }
        else if (cell1.get_north() < cell2.get_north())
        {
            return false;
        }

        return cell1.get_east() < cell2.get_east();
    }
};

class RowCellComparator
{
public:
    bool operator()(const Cell& cell1, const Cell& cell2)
    {
        return cell1.get_priority() > cell2.get_priority();
    }
};

#endif // _CELL_COMPARATORS_H_
