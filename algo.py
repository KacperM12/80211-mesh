import networkx as nx
import matplotlib.pyplot as plt
import math
import json 

class Node:
    def __init__(self, node_id, x, y, uplink_bw=0):
        self.node_id = node_id
        self.x = x  
        self.y = y  
        self.uplink_bw = uplink_bw  
        self.mesh_neighbors = {}    
        
        self.role = 'unknown'       
        self.next_hop = None        
        self.path_cost = float('inf') 

    def add_neighbor(self, neighbor_id, bandwidth):
        self.mesh_neighbors[neighbor_id] = bandwidth

    def reset_state(self):
        """Resetuje stan rutingu węzła przed kolejnym krokiem symulacji."""
        self.mesh_neighbors = {}
        self.role = 'unknown'
        self.next_hop = None
        self.path_cost = float('inf')


def calculate_distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def calculate_link_cost(bandwidth):
    if bandwidth <= 0:
        return float('inf')
    return 100.0 / bandwidth

def run_mesh_routing_algorithm(nodes):
    unvisited = set(nodes.keys())
    
    # 1. FAZA INICJALIZACJI
    for n_id, node in nodes.items():
        if node.uplink_bw > 0:
            node.path_cost = calculate_link_cost(node.uplink_bw)
            node.role = 'gateway'
            node.next_hop = 'SERVER (Uplink)' 
        else:
            node.path_cost = float('inf')
            
    # 2. FAZA ROUTINGU (Dijkstra)
    while unvisited:
        available_nodes = {id: nodes[id] for id in unvisited if nodes[id].path_cost != float('inf')}
        if not available_nodes:
            break
            
        current_id = min(available_nodes, key=lambda id: nodes[id].path_cost)
        current_node = nodes[current_id]
        unvisited.remove(current_id)
        
        for neighbor_id, mesh_bw in current_node.mesh_neighbors.items():
            link_cost = calculate_link_cost(mesh_bw)
            total_cost = current_node.path_cost + link_cost
            
            if total_cost < nodes[neighbor_id].path_cost:
                nodes[neighbor_id].path_cost = total_cost
                nodes[neighbor_id].next_hop = current_id
                if nodes[neighbor_id].uplink_bw == 0:
                    nodes[neighbor_id].role = 'endpoint'

    # 3. FAZA RÓL
    active_next_hops = set(node.next_hop for node in nodes.values() if node.next_hop not in [None, 'SERVER (Uplink)'])
    
    for n_id, node in nodes.items():
        if node.role == 'gateway' and node.next_hop != 'SERVER (Uplink)':
             if n_id in active_next_hops:
                 node.role = 'relay'
             else:
                 node.role = 'endpoint'
        elif node.role == 'endpoint' and n_id in active_next_hops:
            node.role = 'relay'
            
        if node.path_cost == float('inf'):
            node.role = 'isolated'

def visualize_network(nodes, edges_config, title="Topologia Sieci", filename="wykres.png"):
    """Zapisuje wizualizację do pliku graficznego."""
    G = nx.Graph()
    pos = {}
    node_colors = []
    labels = {}
    
    color_map = {
        'gateway': '#2ecc71',  
        'relay': '#f1c40f',    
        'endpoint': '#3498db', 
        'isolated': '#e74c3c', 
        'unknown': 'grey'
    }

    for n_id, node in sorted(nodes.items()):
        G.add_node(n_id)
        pos[n_id] = (node.x, node.y)
        node_colors.append(color_map.get(node.role, 'grey'))
        cost_str = f"{node.path_cost:.1f}" if node.path_cost != float('inf') else "X"
        labels[n_id] = f"{n_id}\n({node.role})\nC:{cost_str}"

    edge_labels = {}
    active_edges = []
    inactive_edges = []

    for u, v, bw in edges_config:
        G.add_edge(u, v, weight=bw)
        edge_labels[(u, v)] = f"{bw}M"
        is_active = (nodes[u].next_hop == v) or (nodes[v].next_hop == u)
        if is_active:
            active_edges.append((u, v))
        else:
            inactive_edges.append((u, v))

    plt.figure(figsize=(12, 9))
    plt.title(title, fontsize=16, fontweight='bold')
    
    nx.draw_networkx_nodes(G, pos, node_size=2500, node_color=node_colors, edgecolors='black', linewidths=1.5)
    nx.draw_networkx_labels(G, pos, labels=labels, font_size=10, font_family='sans-serif', font_weight='bold')
    nx.draw_networkx_edges(G, pos, edgelist=inactive_edges, width=1.0, alpha=0.3, edge_color='grey', style='dashed')
    
    for u, node in nodes.items():
        v = node.next_hop
        if v and v != 'SERVER (Uplink)' and v in nodes:
             plt.arrow(node.x, node.y, (nodes[v].x - node.x)*0.7, (nodes[v].y - node.y)*0.7, 
                       head_width=3, head_length=5, fc='black', ec='black', width=0.5, alpha=0.8, length_includes_head=True)

    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=9, font_color='blue', label_pos=0.3)

    import matplotlib.patches as mpatches
    legend_patches = [mpatches.Patch(color=c, label=l.capitalize()) for l, c in color_map.items() if l != 'unknown']
    plt.legend(handles=legend_patches, title="Role Węzłów", loc='upper left')

    plt.grid(True, linestyle='--', alpha=0.5)
    plt.xlabel("Dystans X [m]")
    plt.ylabel("Dystans Y [m]")
    plt.axis('equal') 
    
    # Zapis i ZAMKNIĘCIE wykresu z pamięci (bardzo ważne w pętli!)
    plt.savefig(filename, bbox_inches='tight')
    plt.close()
    print(f"Zapisano klatkę symulacji: {filename}")


def auto_generate_edges(nodes, max_range_meters=100):
    edges_config = []
    node_keys = list(nodes.keys())
    
    for i in range(len(node_keys)):
        for j in range(i + 1, len(node_keys)):
            n1 = nodes[node_keys[i]]
            n2 = nodes[node_keys[j]]
            
            distance = math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
            
            if distance <= max_range_meters:
                # Opcjonalnie: Przepustowość spada wraz z odległością
                if distance < max_range_meters / 2:
                    bandwidth = 50
                else:
                    bandwidth = 10
                    
                edges_config.append((n1.node_id, n2.node_id, bandwidth))
                
    return edges_config


def load_topology_from_json(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    nodes = {}
    for n in data.get('nodes', []):
        nodes[n['id']] = Node(n['id'], x=n['x'], y=n['y'], uplink_bw=n['uplink_bw'])
        
    return nodes, []


# --- URUCHOMIENIE DYNAMICZNEJ SYMULACJI ---
if __name__ == "__main__":
    
    config_file = 'topology.json'
    print(f"Wczytywanie węzłów z pliku: {config_file}...")
    try:
        nodes, _ = load_topology_from_json(config_file)
    except FileNotFoundError:
        print(f"Błąd: Nie znaleziono pliku {config_file}!")
        exit()

    # Definicja dynamicznych scenariuszy (kroków czasowych)
    scenarios = [
        {
            "step": 1,
            "desc": "Krok 1: Stan normalny (Dobry zasieg, N1 dziala)",
            "wifi_range": 80,
            "n1_uplink": 100,
            "n2_uplink": 5
        },
        {
            "step": 2,
            "desc": "Krok 2: Awaria swiatlowodu w N1! Siec musi zmienic bramke.",
            "wifi_range": 80,
            "n1_uplink": 0,    # N1 traci internet
            "n2_uplink": 5
        },
        {
            "step": 3,
            "desc": "Krok 3: Pogorszenie warunkow radiowych (Zasieg spada do 45m).",
            "wifi_range": 45,  # Drastyczny spadek zasięgu
            "n1_uplink": 0,
            "n2_uplink": 5
        },
        {
            "step": 4,
            "desc": "Krok 4: N1 naprawione, powrot do stanu normalnego.",
            "wifi_range": 80,
            "n1_uplink": 100,  # N1 wraca do gry
            "n2_uplink": 5
        }
    ]

    print("\n--- ROZPOCZĘCIE DYNAMICZNEJ SYMULACJI ---")
    
    for scen in scenarios:
        print(f"\nUruchamianie: {scen['desc']}")
        
        # 1. Zastosowanie zmieniających się warunków
        if 'N1' in nodes: nodes['N1'].uplink_bw = scen['n1_uplink']
        if 'N2' in nodes: nodes['N2'].uplink_bw = scen['n2_uplink']
        current_range = scen['wifi_range']
        
        # 2. Reset stanu węzłów (czyszczenie starych tras)
        for node in nodes.values():
            node.reset_state()
            
        # 3. Przeliczenie fizycznych połączeń (bo zasięg mógł się zmienić)
        edges_config = auto_generate_edges(nodes, max_range_meters=current_range)
        
        for u, v, bw in edges_config:
            if u in nodes and v in nodes:
                nodes[u].add_neighbor(v, bw)
                nodes[v].add_neighbor(u, bw)
                
        # 4. Uruchomienie algorytmu (Self-Healing)
        run_mesh_routing_algorithm(nodes)
        
        # 5. Wygenerowanie i zapis klatki symulacji
        filename = f"symulacja_krok_{scen['step']}.png"
        visualize_network(nodes, edges_config, title=scen['desc'], filename=filename)

    print("\nSymulacja zakończona. Wygenerowano serię obrazów PNG pokazujących adaptację sieci.")