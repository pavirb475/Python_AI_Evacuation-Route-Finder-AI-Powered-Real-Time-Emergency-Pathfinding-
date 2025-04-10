# Import required libraries
import networkx as nx  # For graph data structures and algorithms
import pygame  # For graphics and user interface
from pygame.locals import *  # For pygame constants like QUIT, KEYDOWN etc
import heapq  # For priority queue implementation in pathfinding
import math  # For mathematical operations like distance calculation

# Define window dimensions and visual properties
WINDOW_WIDTH, WINDOW_HEIGHT = 1200, 775  # Size of application window
NODE_SIZE = 20  # Radius of nodes in pixels

# Define colors as RGB tuples for various UI elements
WHITE, BLACK, RED, BLUE, GREEN, GRAY, YELLOW = (255,255,255), (0,0,0), (255,0,0), (0,0,255), (0,255,0), (128,128,128), (255,255,0)
TEXT_SIZE = 24  # Font size for text elements

# Initialize pygame and create window
pygame.init()  # Initialize all pygame modules # must be called before any other pygame functions are used.
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))  # Create main window #function call from the pygame.display module
pygame.display.set_caption("Evacuation Route Finder")  # Set window title
clock = pygame.time.Clock()  # For controlling frame rate
font = pygame.font.Font(None, TEXT_SIZE)  # Initialize font for text rendering

def find_path(graph, start, end):
    """
    Implements Dijkstra's algorithm to find shortest path between start and end nodes
    Takes into account edge weights, node delays, obstacles and blocked edges
    Returns the path and total time taken
    """
    queue = [(0, start)]  # Priority queue with (cost, node) pairs
    visited, costs = {}, {start: 0}  # Track visited nodes and costs to reach nodes
    while queue:
        _, current = heapq.heappop(queue)  # Get node with lowest cost #removes and returns the smallest item from the heap (queue). 
        if current == end: break  # Found destination
        for next_node in graph.neighbors(current):  # Check all neighbors of current node
            # Skip if node is obstacle or edge is blocked
            if next_node in obstacles or (current, next_node) in blocked_edges or (next_node, current) in blocked_edges:
                continue
            # Calculate total cost including edge weight and node delay
            cost = costs[current] + graph.edges[current, next_node]['weight'] + graph.nodes[next_node].get('delay', 0)
            # Update cost if found better path
            if next_node not in costs or cost < costs[next_node]:
                costs[next_node], visited[next_node] = cost, current  # Update costs and visited nodes
                heapq.heappush(queue, (cost, next_node))  # Add to priority queue
    # Reconstruct path from end to start
    path = []  # Initialize empty path
    node = end  # Start from end node
    while node != start:  # Work backwards until reaching start
        path.append(node)  # Add current node to path
        node = visited.get(node)  # Get previous node
        if not node: return [], 0  # No path found
    return [start] + path[::-1], costs.get(end, 0)  # Return path and total cost

def make_default_graph():
    """
    Creates a default graph layout with predefined nodes and edges
    Returns a NetworkX graph with positions, labels and weights
    """
    g = nx.Graph()  # Create empty graph #nx.Graph() is a class in NetworkX for creating undirected graphs.
    # Define node positions and labels (A1-A5, B1-B5, etc)
    nodes = {
        (100,100): "A1", (300,100): "A2", (500,100): "A3", (700,100): "A4", (900,100): "A5",  # Top row
        (100,250): "B1", (300,250): "H1", (500,250): "H2", (700,250): "H3", (900,250): "B5",  # Second row
        (100,400): "C1", (300,400): "H4", (500,400): "H5", (700,400): "H6", (900,400): "C5",  # Third row
        (100,600): "E1", (500,600): "E2", (900,600): "E3"  # Bottom row (exits)
    }
    # Add nodes with their properties
    for pos, label in nodes.items():
        g.add_node(pos, pos=pos, label=label, delay=0)  # Add each node with position, label and zero delay
    
    # Add horizontal edges with weight 2
    edges = [((x,y), (x+200,y), 2) for y in [100,250,400] for x in range(100,900,200)]
    # Add vertical edges with weight 3
    edges += [((x,y), (x,y+150), 3) for x in range(100,1000,200) for y in [100,250]]
    # Add long vertical edges with weight 4
    edges += [((x,400), (x,600), 4) for x in [100,500,900]]
    # Add diagonal edges with weight 4
    edges += [((300,250), (500,400), 4), ((700,250), (500,400), 4), 
             ((300,400), (500,600), 4), ((700,400), (500,600), 4)]
    # Add all edges to graph
    for e in edges:
        g.add_edge(e[0], e[1], weight=e[2])  # Add each edge with its weight #Adds all the edges defined in the edges list to the graph g
    return g  # Return completed graph

def draw(graph, paths, times, buttons, mode="view", selected=None, drawing=None):
    """selected=None: A node currently selected (if any).     drawing=None: Temporary edge being drawn (if any).

    Main rendering function that draws the entire interface
    Handles drawing nodes, edges, paths, labels and UI elements
    """
    screen.fill((240,240,245))  # Light gray background
    
    # Draw all edges
    for e in graph.edges:
        # Red if edge is blocked, gray otherwise
        color = RED if e in blocked_edges or (e[1],e[0]) in blocked_edges else GRAY
        start_pos = graph.nodes[e[0]]['pos']  # Get start position
        end_pos = graph.nodes[e[1]]['pos']  # Get end position
        pygame.draw.line(screen, color, start_pos, end_pos, 3)  # Draw edge line

    # Draw evacuation paths with different colors and offsets
    for path, color, offset in zip(paths, [RED,BLUE,YELLOW], [(0,0),(4,4),(-4,-4)]):  #paths: List of paths to draw.
        if path:  # Only draw if path exists
            for i in range(len(path)-1):  # Draw each segment
                start = tuple(x+o for x,o in zip(graph.nodes[path[i]]['pos'], offset))  # Offset start point #path[i] accesses the i-th node in the list path. #graph.nodes[path[i]]['pos'] fetches the position (the value of the 'pos' key) of the node path[i]. This is assumed to be a tuple of coordinates, like (x, y).
                end = tuple(x+o for x,o in zip(graph.nodes[path[i+1]]['pos'], offset))  # Offset end point
                pygame.draw.line(screen, color, start, end, 5)  # Draw path segment

    # Draw edge weights
    for e in graph.edges:
        start_pos = graph.nodes[e[0]]['pos']  # Get start position
        end_pos = graph.nodes[e[1]]['pos']  # Get end position
        weight = graph.edges[e[0], e[1]]['weight']  # Get edge weight
        mid_x = (start_pos[0] + end_pos[0]) // 2  # Calculate midpoint x
        mid_y = (start_pos[1] + end_pos[1]) // 2  # Calculate midpoint y
        weight_text = font.render(str(weight), True, BLACK)  # Render weight text  #This line converts the weight value into text and prepares it for display.
        weight_rect = weight_text.get_rect(center=(mid_x, mid_y))  # Get text rectangle
        pygame.draw.rect(screen, WHITE, weight_rect.inflate(10,10))  # Draw white background  # Inflates (increases) the size of the rectangle by 10 pixels in both width and height. This gives some padding 
        screen.blit(weight_text, weight_rect)  # Draw weight text

    # Draw nodes with appropriate colors
    for node, data in graph.nodes(data=True):  #Returns a list of nodes, each as a tuple: (node, data).
        # Red for obstacles, green for exits, blue for regular nodes
        color = RED if node in obstacles else GREEN if "E" in data['label'] else BLUE   #node is the unique identifier of the node (e.g., its coordinates (x, y)). data is a dictionary containing properties of the node, like its position (pos), label (label), and delay (delay).
        if node == selected: color = YELLOW  # Yellow for selected node
        pygame.draw.circle(screen, color, data['pos'], NODE_SIZE)  # Draw node circle
        
        # Draw node labels and delay times
        for text, yoffset in [(data['label'],-45), (f"d:{data['delay']}s" if data['delay']>0 else "",45)]:
            if text:  # Only draw if text exists
                surf = font.render(text, True, BLACK if yoffset<0 else GRAY)  # Render text
                rect = surf.get_rect(center=(data['pos'][0], data['pos'][1]+yoffset))  # Position text
                pygame.draw.rect(screen, WHITE, rect.inflate(10,10))  # Draw white background
                screen.blit(surf, rect)  # Draw text

    # Draw temporary edge while creating new connection
    if drawing:
        pygame.draw.line(screen, YELLOW, graph.nodes[drawing[0]]['pos'], pygame.mouse.get_pos(), 3)
    
    # Draw bottom panel
    pygame.draw.rect(screen, (220,220,225), (0,WINDOW_HEIGHT-150,WINDOW_WIDTH,150))  # Panel background
    pygame.draw.line(screen, GRAY, (0,WINDOW_HEIGHT-150), (WINDOW_WIDTH,WINDOW_HEIGHT-150), 2)  # Top border

    # Draw panel content based on mode
    if mode == "view":
        # Draw evacuation times
        screen.blit(font.render("Times:", True, BLACK), (20, WINDOW_HEIGHT-140))  # Draw "Times:" header
        for i,t in enumerate(times):  # Draw each evacuation time
            if t > 0:
                screen.blit(font.render(f"Exit {i+1}: {t:.1f}s", True, BLACK), (40, WINDOW_HEIGHT-110+i*30))

        # Draw start point selection buttons
        if buttons:
            screen.blit(font.render("Start:", True, BLACK), (WINDOW_WIDTH//2-font.size("Start:")[0]//2, WINDOW_HEIGHT-140))
            x,y,w,h = WINDOW_WIDTH//2-250, WINDOW_HEIGHT-110, 60, 30  # Button dimensions
            for i,(pos,btn) in enumerate(buttons.items()):
                rect = pygame.Rect(x+(i%8)*(w+10), y+(i//8)*(h+10), w, h)  # Calculate button position
                pygame.draw.rect(screen, GREEN if btn['selected'] else GRAY, rect)  # Draw button
                pygame.draw.rect(screen, BLACK, rect, 1)  # Draw button border
                text = font.render(btn['label'], True, BLACK)  # Render button text
                screen.blit(text, text.get_rect(center=rect.center))  # Draw button text
                btn['rect'] = rect  # Store button rectangle for click detection

        # Draw help text
        help_text = ["Left Click: Toggle obstacles", "Right Click: Block/Unblock path", 
                    "Scroll: Add/Remove delay", "ESC: Exit"]
        screen.blit(font.render("Help:", True, BLACK), (WINDOW_WIDTH-300, WINDOW_HEIGHT-140))  # Draw help header
        for i,line in enumerate(help_text):  # Draw each help line
            screen.blit(font.render(line, True, BLACK), (WINDOW_WIDTH-280, WINDOW_HEIGHT-110+i*30))
    else:
        # Draw edit mode help text
        screen.blit(font.render("Edit Mode Controls:", True, BLACK), (20, WINDOW_HEIGHT-140))  # Draw edit mode header
        left = ["Left Click: Add new node", "Right Click: Add exit node", "D: Delete selected node"]  # Left column text
        right = ["E: Add edge between nodes", "B: Toggle node blocking", "ESC: Exit edit mode"]  # Right column text
        for i,(l,r) in enumerate(zip(left,right)):  # Draw both columns
            screen.blit(font.render(l, True, BLACK), (40, WINDOW_HEIGHT-110+i*25))  # Draw left text
            screen.blit(font.render(r, True, BLACK), (WINDOW_WIDTH//2, WINDOW_HEIGHT-110+i*25))  # Draw right text
    
    pygame.display.flip()  # Update display

def get_node(graph, pos):
    """
    Returns node at given position if within NODE_SIZE radius
    Returns None if no node found
    """
    return next((n for n in graph.nodes() if math.dist(graph.nodes[n]['pos'], pos) <= NODE_SIZE), None)  # Find closest node

def get_edge(graph, pos):
    """
    Returns edge near given position if within 5 pixels
    Returns None if no edge found
    Uses point-to-line distance calculation
    """
    for e in graph.edges():  # Check each edge
        start, end = graph.nodes[e[0]]['pos'], graph.nodes[e[1]]['pos']  # Get edge endpoints
        length = math.dist(start, end)  # Calculate edge length
        if length == 0: continue  # Skip zero-length edges
        # Calculate closest point on line
        t = max(0, min(1, sum((p-s)*(e-s) for p,s,e in zip(pos,start,end))/length**2))
        if math.dist(pos, (start[0]+t*(end[0]-start[0]), start[1]+t*(end[1]-start[1]))) <= 5:
            return e  # Return edge if close enough
    return None  # No edge found

def get_weight():
    """
    Shows popup to get edge weight from user
    Returns weight (1-9) or None if cancelled
    """
    popup = pygame.Surface((200, 100))  # Create popup surface
    popup.fill(WHITE)  # Fill with white
    pygame.draw.rect(popup, BLACK, popup.get_rect(), 2)  # Draw border
    popup.blit(font.render("Weight (1-9):", True, BLACK), (10, 10))  # Draw prompt
    while True:
        screen.blit(popup, ((WINDOW_WIDTH-200)//2, (WINDOW_HEIGHT-100)//2))  # Draw popup
        pygame.display.flip()  # Update display
        event = pygame.event.wait()  # Wait for input
        if event.type == KEYDOWN:  # Handle key press
            if K_1 <= event.key <= K_9: return event.key - K_0  # Return number key pressed
            if event.key == K_ESCAPE: return None  # Cancel if escape pressed

def edit(graph):
    """
    Edit mode for creating/modifying graph
    Allows adding/removing nodes and edges
    """
    count, selected, drawing = 1, None, None  # Initialize state variables
    while True:
        clock.tick(60)  # Limit to 60 FPS
        draw(graph, [], [], {}, "edit", selected, drawing)  # Draw edit mode interface
        for event in pygame.event.get():  # Handle events
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                return graph  # Exit edit mode
            elif event.type == MOUSEBUTTONDOWN and event.pos[1] < WINDOW_HEIGHT-150:  # Handle mouse clicks
                node = get_node(graph, event.pos)  # Get clicked node
                if event.button == 1:  # Left click
                    if not node:  # Add new node
                        graph.add_node(event.pos, pos=event.pos, label=f"N{count}", delay=0)
                        count += 1
                    elif drawing:  # Complete edge drawing
                        if node != drawing[0]:  # Don't connect node to itself
                            if (w := get_weight()): graph.add_edge(drawing[0], node, weight=w)
                        drawing = selected = None
                    else:  # Start edge drawing
                        selected, drawing = node, (node, None)
                elif event.button == 3 and not node:  # Right click - add exit
                    num = sum(1 for n in graph.nodes if 'E' in graph.nodes[n]['label']) + 1
                    graph.add_node(event.pos, pos=event.pos, label=f"E{num}", delay=0)
            elif event.type == KEYDOWN:  # Handle key presses
                pos = pygame.mouse.get_pos()  # Get mouse position
                node = get_node(graph, pos)  # Get node under cursor
                if event.key == K_d and node:  # Delete node
                    graph.remove_node(node)
                    selected = drawing = None
                elif event.key == K_e:  # Cancel edge drawing
                    selected = drawing = None
                elif event.key == K_b and node:  # Toggle obstacle
                    if node in obstacles: obstacles.remove(node)
                    else: obstacles.add(node)

def main():
    """
    Main program loop
    Handles mode switching between select, edit and view modes
    """
    graph = start = None  # Initialize variables
    mode = "select"  # Start in select mode
    while True:
        clock.tick(60)  # Limit to 60 FPS
        if mode == "select":
            # Draw mode selection screen
            screen.fill((240,240,245))  # Clear screen
            text = font.render("Choose Graph", True, BLACK)  # Render title
            screen.blit(text, text.get_rect(center=(WINDOW_WIDTH//2, WINDOW_HEIGHT//2-100)))  # Draw title
            
            default = pygame.Rect(WINDOW_WIDTH//2-225, WINDOW_HEIGHT//2-30, 200, 60)  # Default button
            custom = pygame.Rect(WINDOW_WIDTH//2+25, WINDOW_HEIGHT//2-30, 200, 60)  # Custom button
            
            for btn,txt in [(default,"Default"),(custom,"Custom")]:  # Draw buttons
                pygame.draw.rect(screen, GRAY, btn)  # Button background
                pygame.draw.rect(screen, BLACK, btn, 2)  # Button border
                text = font.render(txt, True, BLACK)  # Button text
                screen.blit(text, text.get_rect(center=btn.center))  # Draw text
            
            pygame.display.flip()  # Update display
            
            # Handle mode selection events
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    pygame.quit()  # Exit program
                    return
                elif event.type == MOUSEBUTTONDOWN:
                    if default.collidepoint(event.pos):  # Default graph selected
                        graph, mode = make_default_graph(), "view"
                    elif custom.collidepoint(event.pos):  # Custom graph selected
                        graph, mode = edit(nx.Graph()), "view"
        
        elif mode == "view":
            # Initialize view mode if needed
            if not start:
                start = next(iter(graph.nodes()))  # Set initial start node
                exits = [n for n in graph.nodes() if 'E' in graph.nodes[n]['label']]  # Get exit nodes
                buttons = {pos: {'label': data['label'], 'selected': pos == start, 'rect': None} 
                          for pos, data in graph.nodes(data=True) if 'E' not in data['label']}  # Create buttons
                paths = []  # Initialize paths
                times = []  # Initialize times
                for exit in exits:  # Calculate initial paths
                    path, time = find_path(graph, start, exit)
                    paths.append(path)
                    times.append(time)
            
            draw(graph, paths, times, buttons)  # Draw view mode interface
            
            # Handle view mode events
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    pygame.quit()  # Exit program
                    return
                elif event.type == MOUSEBUTTONDOWN:
                    pos = event.pos  # Get click position
                    
                    # Handle start point selection
                    for node_pos, btn in buttons.items():
                        if btn['rect'] and btn['rect'].collidepoint(pos):  # Check if button clicked
                            start = node_pos  # Set new start node
                            for b in buttons.values(): b['selected'] = False  # Deselect all buttons
                            btn['selected'] = True  # Select clicked button
                            paths = []  # Reset paths
                            times = []  # Reset times
                            for exit in exits:  # Recalculate paths
                                path, time = find_path(graph, start, exit)
                                paths.append(path)
                                times.append(time)
                            break
                    
                    # Handle node interactions
                    node = get_node(graph, pos)  # Get clicked node
                    if node:
                        if event.button == 1:  # Toggle obstacle
                            if node in obstacles: obstacles.remove(node)
                            else: obstacles.add(node)
                        elif event.button == 4:  # Increase delay
                            graph.nodes[node]['delay'] += 1
                        elif event.button == 5:  # Decrease delay
                            graph.nodes[node]['delay'] = max(0, graph.nodes[node]['delay']-1)
                    else:
                        # Handle edge blocking
                        edge = get_edge(graph, pos)  # Get clicked edge
                        if edge and event.button in [1,3]:  # Toggle edge blocking
                            if edge in blocked_edges or (edge[1],edge[0]) in blocked_edges:
                                blocked_edges.discard(edge)
                                blocked_edges.discard((edge[1],edge[0]))
                            else:
                                blocked_edges.add(edge)
                    
                    # Recalculate paths if graph changed
                    if node or edge:
                        paths = []  # Reset paths
                        times = []  # Reset times
                        for exit in exits:  # Recalculate all paths
                            path, time = find_path(graph, start, exit)
                            paths.append(path)
                            times.append(time)

if __name__ == "__main__":
    obstacles = set()  # Set to store obstacle nodes
    blocked_edges = set()  # Set to store blocked edges
    main()  # Start program