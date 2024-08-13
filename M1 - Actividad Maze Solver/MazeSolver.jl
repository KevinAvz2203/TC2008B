import Pkg; Pkg.add("FileIO")
import Pkg; Pkg.add("ImageMagick")

using Agents, Agents.Pathfinding
using FileIO
using ImageMagick

@agent struct Walker(GridAgent{2}) end

function initialize_model(maze_map)
    # Load the maze from the image file. White values can be 
    # identified by a non-zero red component
    maze = BitArray(map(x -> x.r > 0, maze_map))

    # The size of the space is the size of the maze
    space = GridSpace(size(maze); periodic = false)

    # Create a pathfinder using the AStar algorithm by providing the
    # space and specifying
    # the `walkmap` parameter for the pathfinder.
    pathfinder = AStar(space; walkmap=maze, diagonal_movement=false)
    model = StandardABM(Walker, space; agent_step!)

    # Place a walker at the start of the maze
    add_agent!((5, 152), model)

    # The walker's movement target is the end of the maze.
    plan_route!(model[1], (318, 170), pathfinder)

    return model, pathfinder
end

agent_step!(agent, model) = move_along_route!(agent, model, pathfinder)

# Our sample walkmap
map_url = "sigmaMaze.bmp"
maze_map = load(map_url);
model, pathfinder = initialize_model(maze_map)

using CairoMakie

abmvideo(
    "mazeK.mp4",
    model;
    figurekwargs = (size =(322, 322),), # Maze Size
    frames=1500,
    framerate=60,
    agent_color=:green,
    agent_size=10,
    heatarray = _ -> pathfinder.walkmap,
    add_colorbar = false,
)
