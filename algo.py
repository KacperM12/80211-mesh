import networkx as nx
import matplotlib.pyplot as plt
import math
import json 
import math


class Node:
    def __init__(self, node_id, x, y, uplink_bw=0):
        self.node_id = node_id
        self.x = x  # Współrzędna X w metrach
        self.y = y  # Współrzędna Y w metrach
        # Przepustowość bezpośrednio do serwera (np. 5G/Ethernet) w Mbps. 0 = brak.
        self.uplink_bw = uplink_bw  
        self.mesh_neighbors = {}    # Słownik: id_sasiada -> przepustowosc_linku_mesh (Mbps)
        
        # Atrybuty ustalane przez algorytm
        self.role = 'unknown'       # 'gateway', 'relay', 'endpoint'
        self.next_hop = None        # Przez kogo wysyłać dane
        self.path_cost = float('inf') # Sumaryczny "koszt" dotarcia do serwera

    def add_neighbor(self, neighbor_id, bandwidth):
        self.mesh_neighbors[neighbor_id] = bandwidth

def calculate_distance(node1, node2):
    """Liczy dystans euklidesowy między węzłami."""
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def calculate_link_cost(bandwidth):
    """Im wyższa przepustowość, tym niższy koszt."""
    if bandwidth <= 0:
        return float('inf')
    return 100.0 / bandwidth

def run_mesh_routing_algorithm(nodes):
    """Algorytm Dijkstry wyznaczania ról i tras."""
    unvisited = set(nodes.keys())
    
    # 1. FAZA INICJALIZACJI: Wstępnie oznacz bramki (Gateway)
    for n_id, node in nodes.items():
        if node.uplink_bw > 0:
            node.path_cost = calculate_link_cost(node.uplink_bw)
            node.role = 'gateway'
            node.next_hop = 'SERVER (Uplink)' 
        else:
            node.path_cost = float('inf')
            
    # 2. FAZA ROUTINGU: Propagacja najlepszych tras (Dijkstra)
    while unvisited:
        # Wybierz nieodwiedzony węzeł o najmniejszym koszcie
        available_nodes = {id: nodes[id] for id in unvisited if nodes[id].path_cost != float('inf')}
        if not available_nodes:
            break
            
        current_id = min(available_nodes, key=lambda id: nodes[id].path_cost)
        current_node = nodes[current_id]
        unvisited.remove(current_id)
        
        # Sprawdzamy sąsiadów
        for neighbor_id, mesh_bw in current_node.mesh_neighbors.items():
            link_cost = calculate_link_cost(mesh_bw)
            total_cost = current_node.path_cost + link_cost
            
            if total_cost < nodes[neighbor_id].path_cost:
                nodes[neighbor_id].path_cost = total_cost
                nodes[neighbor_id].next_hop = current_id
                # Tymczasowo oznacz jako endpoint, jeśli nie jest bramką z Fazy 1
                if nodes[neighbor_id].uplink_bw == 0:
                    nodes[neighbor_id].role = 'endpoint'

    # 3. FAZA RÓL (Poprawiona): Doprecyzowanie ról
    # Kto jest aktywnie wykorzystywany jako przekaźnik?
    active_next_hops = set(node.next_hop for node in nodes.values() if node.next_hop not in [None, 'SERVER (Uplink)'])
    
    for n_id, node in nodes.items():
        # Poprawka logiki N2: Jeśli bramka wysyła przez mesh, staje się endpointem/relayem
        if node.role == 'gateway' and node.next_hop != 'SERVER (Uplink)':
             # Jeśli ktoś przez niego wysyła, jest przekaźnikiem
             if n_id in active_next_hops:
                 node.role = 'relay'
             else:
                 node.role = 'endpoint'

        # Jeśli zwykły węzeł przekazuje dane innych
        elif node.role == 'endpoint' and n_id in active_next_hops:
            node.role = 'relay'
            
        if node.path_cost == float('inf'):
            node.role = 'isolated'

def visualize_network(nodes, edges_config, title="Topologia Sieci Mesh"):
    """Tworzy wizualizację graficzną sieci przy użyciu NetworkX i Matplotlib."""
    G = nx.Graph()
    
    # 1. Dodawanie węzłów i ustalanie ich pozycji na podstawie (X, Y)
    pos = {}
    node_colors = []
    labels = {}
    
    # Definicja kolorów dla ról
    color_map = {
        'gateway': '#2ecc71',  # Zielony (działająca bramka)
        'relay': '#f1c40f',    # Żółty (przekaźnik)
        'endpoint': '#3498db', # Niebieski (tylko wysyła)
        'isolated': '#e74c3c', # Czerwony (odcięty)
        'unknown': 'grey'
    }

    print("\n--- Pozycje węzłów i ostateczne role ---")
    for n_id, node in sorted(nodes.items()):
        G.add_node(n_id)
        pos[n_id] = (node.x, node.y)
        node_colors.append(color_map.get(node.role, 'grey'))
        
        # Etykieta: ID + Rola + Koszt
        cost_str = f"{node.path_cost:.1f}" if node.path_cost != float('inf') else "X"
        labels[n_id] = f"{n_id}\n({node.role})\nC:{cost_str}"
        print(f"{n_id} ({node.x},{node.y}) -> {node.role}")

    # 2. Dodawanie wszystkich fizycznie możliwych połączeń Mesh
    edge_labels = {}
    
    # Rozróżniamy krawędzie aktywne (używane w routingu) od nieaktywnych
    active_edges = []
    inactive_edges = []

    for u, v, bw in edges_config:
        G.add_edge(u, v, weight=bw)
        edge_labels[(u, v)] = f"{bw}M"
        
        # Sprawdź, czy to połączenie jest wykorzystywane przez routingu w którąkolwiek stronę
        is_active = (nodes[u].next_hop == v) or (nodes[v].next_hop == u)
        if is_active:
            active_edges.append((u, v))
        else:
            inactive_edges.append((u, v))

    # 3. Rysowanie
    plt.figure(figsize=(12, 9))
    plt.title(title, fontsize=16, fontweight='bold')
    
    # Rysuj węzły
    nx.draw_networkx_nodes(G, pos, node_size=2500, node_color=node_colors, edgecolors='black', linewidths=1.5)
    
    # Rysuj etykiety węzłów
    nx.draw_networkx_labels(G, pos, labels=labels, font_size=10, font_family='sans-serif', font_weight='bold')

    # Rysuj nieaktywne krawędzie (szare, przerywane)
    nx.draw_networkx_edges(G, pos, edgelist=inactive_edges, width=1.0, alpha=0.3, edge_color='grey', style='dashed')
    
    # Rysuj aktywne krawędzie routingu (grubsze, czarne, strzałki)
    # NetworkX draw_networkx_edges domyślnie nie rysuje strzałek dla nx.Graph, 
    # ale użyjemy triku, żeby pokazać kierunek "next_hop".
    
    for u, node in nodes.items():
        v = node.next_hop
        if v and v != 'SERVER (Uplink)' and v in nodes:
             # Rysujemy strzałkę od węzła do jego Next Hopa
             plt.arrow(node.x, node.y, (nodes[v].x - node.x)*0.7, (nodes[v].y - node.y)*0.7, 
                       head_width=3, head_length=5, fc='black', ec='black', width=0.5, alpha=0.8, length_includes_head=True)

    # Rysuj etykiety krawędzi (przepustowość)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=9, font_color='blue', label_pos=0.3)

    # Dodaj legendę dla kolorów
    import matplotlib.patches as mpatches
    legend_patches = [mpatches.Patch(color=c, label=l.capitalize()) for l, c in color_map.items() if l != 'unknown']
    plt.legend(handles=legend_patches, title="Role Węzłów", loc='upper left')

    plt.grid(True, linestyle='--', alpha=0.5)
    plt.xlabel("Dystans X [m]")
    plt.ylabel("Dystans Y [m]")
    plt.axis('equal') # Zachowaj proporcje 1:1 metrów
    
    print("\nZamykam okno wykresu, aby zakończyć program.")
    plt.savefig('wykres_topologii.png', bbox_inches='tight')
    print("Zapisano wykres do pliku: wykres_topologii.png")


def auto_generate_edges(nodes, max_range_meters=100):
    """
    Automatycznie generuje połączenia mesh między węzłami, 
    które znajdują się w swoim zasięgu fizycznym.
    """
    edges_config = []
    node_keys = list(nodes.keys())
    
    # Sprawdzamy każdą możliwą parę węzłów
    for i in range(len(node_keys)):
        for j in range(i + 1, len(node_keys)):
            n1 = nodes[node_keys[i]]
            n2 = nodes[node_keys[j]]
            
            # Obliczanie dystansu euklidesowego
            distance = math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
            
            # jeśli są w zasięgu, tworzymy link
            if distance <= max_range_meters:
                # Opcjonalnie: Przepustowość spada wraz z odległością
                # Np. blisko = 50 Mbps, na skraju zasięgu = 5 Mbps
                if distance < max_range_meters / 2:
                    bandwidth = 50
                else:
                    bandwidth = 10
                    
                edges_config.append((n1.node_id, n2.node_id, bandwidth))
                
    return edges_config


def load_topology_from_json(filepath):
    """Wczytuje węzły i krawędzie z pliku JSON."""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    nodes = {}
    edges_config = []
    
    # tworzenie obiektów Node na podstawie JSON
    for n in data.get('nodes', []):
        nodes[n['id']] = Node(n['id'], x=n['x'], y=n['y'], uplink_bw=n['uplink_bw'])
        
    # pobieranie definicji krawędzi
    for e in data.get('edges', []):
        edges_config.append((e['u'], e['v'], e['bw']))
        
    return nodes, edges_config

# uruchonmienie symulacji
if __name__ == "__main__":
    
    # 1. Wczytujemy dane z pliku
    config_file = 'topology.json'
    print(f"Wczytywanie węzłów z pliku: {config_file}...")
    
    try:
        # Pobieramy węzły, ale ignorujemy krawędzie z jsona (używając znaku '_') - zaszla zmiana na automatycznie generowane lącza
        nodes, _ = load_topology_from_json(config_file)
    except FileNotFoundError:
        print(f"Błąd: Nie znaleziono pliku {config_file}!")
        exit()
        

    zasieg_wifi = 80
    print(f"Obliczanie zasięgu fizycznego urządzeń (max {zasieg_wifi}m)...")
    edges_config = auto_generate_edges(nodes, max_range_meters=zasieg_wifi)
        
    # 3. Dodajemy sąsiadów do obiektów Node na podstawie wyliczonych odległości
    for u, v, bw in edges_config:
        if u in nodes and v in nodes:
            nodes[u].add_neighbor(v, bw)
            nodes[v].add_neighbor(u, bw)
        
    # 4. Uruchamiamy algorytm routingu
    run_mesh_routing_algorithm(nodes)
    
    # 5. Wizualizacja
    visualize_network(nodes, edges_config, f"Wizualizacja Zasięgu Mesh (Max {zasieg_wifi}m)")