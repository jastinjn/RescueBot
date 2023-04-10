#ifndef THERMAL_GRID_HPP
#define THERMAL_GRID_HPP

#include <common/point.hpp>
#include <lcmtypes/thermal_grid_t.hpp>
#include <algorithm>
#include <cstdint>
#include <vector>


class ThermalGrid
{
public:
    
    /**
    * Default constructor for OccupancyGrid.
    * 
    * Create an OccupancyGrid with a width and height of 0. metersPerCell is set to 0.05. The global origin is (0,0).
    * 
    * This constructor is intended for use when creating a grid from an LCM message.
    */
    ThermalGrid(void);
    
    /**
    * Constructor for OccupancyGrid.
    * 
    * The origin is (-widthInMeters/2, -heightInMeters/2).
    * 
    * \param    widthInMeters 
    * \param    heightInMeters
    * \param    metersPerCell
    * 
    * \pre  widthInMeters  > 0
    * \pre  heightInMeters > 0
    * \pre  metersPerCell <= widthInMeters
    * \pre  metersPerCell <= heightInMeters
    */
    ThermalGrid(float widthInMeters,
                  float heightInMeters,
                  float metersPerCell);
    
    
    // Accessors for the properties of the grid
    int   widthInCells (void) const { return width_; }
    float widthInMeters(void) const { return width_ * metersPerCell_; }
    
    int   heightInCells (void) const { return height_; }
    float heightInMeters(void) const { return height_ * metersPerCell_; }
    
    float metersPerCell(void) const { return metersPerCell_; }
    float cellsPerMeter(void) const { return cellsPerMeter_; }
    
    Point<float> originInGlobalFrame(void) const { return globalOrigin_; }
    
    void setOrigin(float x, float y);

    /**
    * reset resets all cells into the grid to equal odds. All cells will have logOdds == 0.
    */
    void reset(void);
    
    /**
    * isCellInGrid checks to see if the specified cell is within the boundary of the OccupancyGrid.
    * 
    * This test is equivalent to:
    * 
    *   (0 <= x < widthInCells) && (0 <= y < heightInCells)
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   True if the cell is in the grid boundary as specified above.
    */
    bool isCellInGrid(int x, int y) const;
    
    /**
    * odds retreives the log-odds that a specific cell is occupied.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   The log-odds at cell (x,y). If (x,y) is not contained in the grid, i.e. isCellInGrid(x,y) == false,
    *           then 0 is returned. This is equivalent to the log-odds of an unobserved cell in the map.
    */
    float thermalValue(int x, int y) const;
    
    /**
    * setLogOdds sets the log-odds of occupancy for the specified cell if the cell falls within the grid boundary. If the
    * cell doesn't fall within the boundary, then nothing happens.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \param    logOdds     Log-odds to assign to the cell
    */
    void setThermalValue(int x, int y, float value);
    
    /**
    * operator() provides unchecked access to the cell located at (x,y). If the cell isn't contained in the grid,
    * expect fireworks or a slow, ponderous death.
    * 
    * This version allows direct modification of the logOdds. If you are iterating through the grid
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   A mutable reference to the cell (x,y)'s logOdds.
    */
    float& operator()(int x, int y)       { return cells_[cellIndex(x, y)]; }
    
    /**
    * operator() provides unchecked access to the cell located at (x,y). If the cell isn't contained in the grid,
    * expect fireworks or a slow, ponderous death.
    * 
    * \param    x           x-coordinate of the cell
    * \param    y           y-coordinate of the cell
    * \return   The logOdds of cell (x, y).
    */
    float  operator()(int x, int y) const { return cells_[cellIndex(x, y)]; }

    /**
    * toLCM creates an LCM message from the grid.
    */
    thermal_grid_t toLCM(void) const;
    
    /**
    * fromLCM populates the grid using an LCM message. The current contents of the grid are erased.
    */
    void fromLCM(const thermal_grid_t& gridMessage);
    
    /**
    * saveToFile saves the OccupancyGrid to the specified file.
    * 
    * The format for saving is:
    * 
    *   origin_x origin_y width_in_cells height_in_cells meters_per_cell
    *   (0, 0) (0, 1) . . .
    *           .
    *           .
    *           .
    *   (0, height-1) . . . (width-1, height-1)
    * 
    * where each cell value is stored in ASCII text.
    * 
    * \param    filename            Name of the map to be saved
    * \return   True if the map is successfully saved. False if the file can't be opened or some other I/O error occurs.
    */
    bool saveToFile(const std::string& filename) const;
    
    /**
    * loadFromFile loads the OccupancyGrid from a file in the format specified in saveToFile.
    * 
    * Minimal error checking is performed, so an improperly formatted map file can produce very strange results.
    * 
    * \param    filename            Name of the map to be loaded
    * \return   True if the map is successfully loaded. False if the file can't be opened or some other I/O error occurs.
    */
    bool loadFromFile(const std::string& filename);
    
private:
    
    std::vector<float> cells_;       ///< The actual grid -- stored in row-column order
    
    int width_;                 ///< Width of the grid in cells
    int height_;                ///< Height of the grid in cells
    float metersPerCell_;
    float cellsPerMeter_;
    
    Point<float> globalOrigin_;         ///< Origin of the grid in global coordinates
    
    // Convert between cells and the underlying vector index
    int cellIndex(int x, int y) const { return y*width_ + x; }
};

#endif // MAPPING_OCCUPANCY_GRID_HPP
