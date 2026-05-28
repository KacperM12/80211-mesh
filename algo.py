import networkx as nx
import matplotlib.pyplot as plt
import math
import json 
import random

# === Parametry (wagi) algorytmu z notatki ===
A_WEIGHT = 1.0  # Waga opóźnienia ping
B_WEIGHT = 2.0  # Waga niestabilności jiter'a
C_WEIGHT = 50.0 # Waga zatoru (V_ruch / B_wolne)
D_WEIGHT = 200.0# Waga zaszumienia eteru (U_eter)

C_HOP = 100.0   # k_skok: stały "podatek" od każdego skoku wewnątrz sieci
C_WIFI = 1000.0 # Baza kary za powolny link fizyczny dla K_link
DEFAULT_B_WOLNE_RATIO = 0.8
DEFAULT_B_PHY = 50.0
DEFAULT_U_ETER = 0.1
# ==========================================

class Node:
    def __init__(
        self,
        node_id,
        x,
        y,
        uplink_bw=0,
        is_data_source=False,
        c_tech=None,
        ping_avg=15.0,
        ping_mdev=2.0,
        v_ruch=5.0,
        b_wolne=None,
    ):
        self.node_id = node_id
        self.x = x  
        self.y = y  
        
        # Metryki zewnetrzne (Gateway)
        self.uplink_bw = uplink_bw
        self.c_tech = c_tech if c_tech is not None else (10 if uplink_bw > 0 else 0)
        self.ping_avg = ping_avg
        self.ping_mdev = ping_mdev
        self.v_ruch = v_ruch
        if b_wolne is None:
            self.b_wolne = uplink_bw * DEFAULT_B_WOLNE_RATIO
        else:
            self.b_wolne = b_wolne
        
        self.is_data_source = is_data_source
        self.mesh_neighbors = {}  # {neighbor_id: {'b_phy': val, 'u_eter': val}}  
        
        self.role = 'unknown'       
        self.next_hop = None        
        self.path_cost = float('inf') 

    def add_neighbor(self, neighbor_id, b_phy, u_eter):
        self.mesh_neighbors[neighbor_id] = {'b_phy': b_phy, 'u_eter': u_eter}

    def update_gateway_metrics(
        self,
        uplink_bw=None,
        c_tech=None,
        ping_avg=None,
        ping_mdev=None,
        v_ruch=None,
        b_wolne=None,
    ):
        if uplink_bw is not None:
            self.uplink_bw = uplink_bw
            if b_wolne is None:
                self.b_wolne = uplink_bw * DEFAULT_B_WOLNE_RATIO
        if c_tech is not None:
            self.c_tech = c_tech
        if ping_avg is not None:
            self.ping_avg = ping_avg
        if ping_mdev is not None:
            self.ping_mdev = ping_mdev
        if v_ruch is not None:
            self.v_ruch = v_ruch
        if b_wolne is not None:
            self.b_wolne = b_wolne

    def reset_state(self):
        """Resetuje stan rutingu węzła przed kolejnym krokiem symulacji."""
        self.mesh_neighbors = {}
        self.role = 'unknown'
        self.next_hop = None
        self.path_cost = float('inf')


def calculate_distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def calculate_w_zew(node):
    if node.uplink_bw <= 0:
        return float('inf')
    k_baz = node.c_tech
    k_ping = A_WEIGHT * node.ping_avg
    k_jitter = B_WEIGHT * node.ping_mdev
    b_wolne_safe = node.b_wolne if node.b_wolne > 0.1 else 0.1
    k_zator = C_WEIGHT * (node.v_ruch / b_wolne_safe)
    return k_baz + k_ping + k_jitter + k_zator

def calculate_w_wew(b_phy, u_eter):
    if b_phy <= 0:
        return float('inf')
    k_skok = C_HOP
    k_link = C_WIFI / b_phy
    u_eter_safe = u_eter if u_eter is not None else 0.0
    k_eter = D_WEIGHT * u_eter_safe
    return k_skok + k_link + k_eter

def normalize_link_key(u, v):
    return tuple(sorted((u, v)))

def build_link_update_map(link_updates):
    updates = {}
    for upd in link_updates or []:
        if 'u' not in upd or 'v' not in upd:
            continue
        updates[normalize_link_key(upd['u'], upd['v'])] = upd
    return updates

def build_edges_from_links(links, link_updates=None):
    updates = build_link_update_map(link_updates)
    edges = []
    for link in links:
        u = link['u']
        v = link['v']
        key = normalize_link_key(u, v)
        upd = updates.get(key, {})
        enabled = upd.get('enabled', link.get('enabled', True))
        if not enabled:
            continue
        edges.append({
            'u': u,
            'v': v,
            'b_phy': upd.get('b_phy', link.get('b_phy', DEFAULT_B_PHY)),
            'u_eter': upd.get('u_eter', link.get('u_eter', DEFAULT_U_ETER)),
        })
    return edges

def apply_link_updates(edges, link_updates):
    updates = build_link_update_map(link_updates)
    if not updates:
        return edges
    updated_edges = []
    for edge in edges:
        key = normalize_link_key(edge['u'], edge['v'])
        upd = updates.get(key, {})
        enabled = upd.get('enabled', edge.get('enabled', True))
        if not enabled:
            continue
        new_edge = dict(edge)
        if 'b_phy' in upd:
            new_edge['b_phy'] = upd['b_phy']
        if 'u_eter' in upd:
            new_edge['u_eter'] = upd['u_eter']
        updated_edges.append(new_edge)
    return updated_edges

def run_mesh_routing_algorithm(nodes):
    unvisited = set(nodes.keys())
    
    # 1. FAZA INICJALIZACJI
    for n_id, node in nodes.items():
        if node.uplink_bw > 0:
            node.path_cost = calculate_w_zew(node)  # Bramki (Gateway) na zewnatrz
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
        
        for neighbor_id, link_params in current_node.mesh_neighbors.items():
            link_cost = calculate_w_wew(link_params['b_phy'], link_params['u_eter']) # Siec Mesh wewnatrz
            total_cost = current_node.path_cost + link_cost
            
            if total_cost < nodes[neighbor_id].path_cost:
                nodes[neighbor_id].path_cost = total_cost
                nodes[neighbor_id].next_hop = current_id
                if nodes[neighbor_id].uplink_bw == 0:
                    nodes[neighbor_id].role = 'endpoint'

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

def calculate_link_loads(nodes):
    """Sumuje obciazenie laczy na podstawie tras i v_ruch zrodel danych."""
    node_totals = {}
    for node in nodes.values():
        if node.path_cost == float('inf'):
            node_totals[node.node_id] = 0.0
        elif node.is_data_source and node.uplink_bw == 0:
            node_totals[node.node_id] = max(0.0, node.v_ruch)
        else:
            node_totals[node.node_id] = 0.0

    ordered_nodes = sorted(
        (n for n in nodes.values() if n.path_cost != float('inf')),
        key=lambda n: n.path_cost,
        reverse=True,
    )

    link_loads = {}
    for node in ordered_nodes:
        next_id = node.next_hop
        if not next_id or next_id == 'SERVER (Uplink)':
            continue
        if next_id not in nodes:
            continue
        total = node_totals.get(node.node_id, 0.0)
        if total <= 0:
            continue
        edge_key = tuple(sorted((node.node_id, next_id)))
        link_loads[edge_key] = link_loads.get(edge_key, 0.0) + total
        node_totals[next_id] = node_totals.get(next_id, 0.0) + total

    return link_loads

def visualize_network(nodes, edges_config, title="Topologia Sieci", filename="wykres.png", link_loads=None):
    """Zapisuje wizualizację do pliku graficznego."""
    G = nx.Graph()
    pos = {}
    node_colors = []
    labels = {}
    link_loads = link_loads or {}
    
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
        source_marker = " [ŹRÓDŁO]" if node.is_data_source else ""
        cost_str = f"{node.path_cost:.1f}" if node.path_cost != float('inf') else "X"
        labels[n_id] = f"{n_id}{source_marker}\n({node.role})\nC:{cost_str}"

    edge_labels = {}
    edge_b_phy = {}
    active_edges = set()
    inactive_edges = set()
    added_edges = set()

    for edge in edges_config:
        if isinstance(edge, dict):
            u = edge['u']
            v = edge['v']
            b_phy = edge.get('b_phy', DEFAULT_B_PHY)
        else:
            u, v, b_phy = edge
        edge_key = tuple(sorted((u, v)))
        if edge_key not in added_edges:
            G.add_edge(*edge_key, weight=b_phy)
            added_edges.add(edge_key)
        edge_b_phy[edge_key] = max(edge_b_phy.get(edge_key, 0.0), b_phy)

        is_active = (nodes[u].next_hop == v) or (nodes[v].next_hop == u)
        if is_active:
            active_edges.add(edge_key)
            inactive_edges.discard(edge_key)
        elif edge_key not in active_edges:
            inactive_edges.add(edge_key)

    for edge_key, b_phy in edge_b_phy.items():
        load = link_loads.get(edge_key, 0.0)
        edge_labels[edge_key] = f"{load:g}/{b_phy:g}M"

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


def auto_generate_edges(nodes, max_range_meters=100, default_u_eter=DEFAULT_U_ETER):
    edges_config = []
    node_keys = list(nodes.keys())
    
    for i in range(len(node_keys)):
        for j in range(i + 1, len(node_keys)):
            n1 = nodes[node_keys[i]]
            n2 = nodes[node_keys[j]]
            
            distance = math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
            
            if distance <= max_range_meters:
                # Opcjonalnie: Przepustowość spada wraz z odległością
                bandwidth = simulate_b_phy(distance)
                    
                edges_config.append({
                    'u': n1.node_id,
                    'v': n2.node_id,
                    'b_phy': bandwidth,
                    'u_eter': default_u_eter,
                })
                
    return edges_config


def load_topology_from_json(filepath):
    """Wczytuje węzły z pliku JSON."""
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
        
    nodes = {}
    for n in data.get('nodes', []):
        # Pobieramy status źródła (jeśli nie wpisano w JSON, domyślnie uznajemy za True)
        is_src = n.get('is_data_source', True)
        
        nodes[n['id']] = Node(
            n['id'], 
            x=n['x'], 
            y=n['y'], 
            uplink_bw=n.get('uplink_bw', 0),
            is_data_source=is_src,
            c_tech=n.get('c_tech'),
            ping_avg=n.get('ping_avg', 15.0),
            ping_mdev=n.get('ping_mdev', 2.0),
            v_ruch=n.get('v_ruch', 5.0),
            b_wolne=n.get('b_wolne'),
        )

    links = []
    for link in data.get('links', []):
        u = link.get('u')
        v = link.get('v')
        if not u or not v:
            continue
        links.append({
            'u': u,
            'v': v,
            'b_phy': link.get('b_phy', link.get('bandwidth', DEFAULT_B_PHY)),
            'u_eter': link.get('u_eter', DEFAULT_U_ETER),
            'enabled': link.get('enabled', True),
        })
        
    return nodes, links


def simulate_b_phy(distance):
    """Zwraca prędkość Wi-Fi w oparciu o fizyczną odległość i lekki szum."""
    if distance < 15:
        base_phy = 150.0  # Bardzo blisko, max modulacja
    elif distance < 35:
        base_phy = 54.0   # Średni zasięg
    elif distance < 60:
        base_phy = 11.0   # Daleko, niska modulacja
    else:
        base_phy = 1.0    # Granica zasięgu

    # Szum środowiskowy (Gauss) - wiatr, zakłócenia, np +/- 10%
    noise = random.gauss(0, base_phy * 0.1)
    return max(1.0, base_phy + noise)


def update_environment_physics(nodes):
    """
    Symuluje reakcję środowiska po wyznaczeniu tras:
    Zlicza ruch docierający do bramek i nieliniowo podbija im PING (Bufferbloat).
    """
    # 1. Sprawdźmy, ile ruchu V_ruch z czujników trafia do poszczególnych bramek
    gateway_loads = {n_id: 0.0 for n_id, n in nodes.items() if n.uplink_bw > 0}
    
    for n_id, node in nodes.items():
        if node.is_data_source and node.uplink_bw == 0:
            # Śledzimy trasę od czujnika po "next_hop" aż trafimy do bramki
            current = node
            visited = set()
            while current.next_hop and current.next_hop != 'SERVER (Uplink)':
                if current.node_id in visited:
                    break # Zabezpieczenie przed pętlą
                visited.add(current.node_id)
                current = nodes.get(current.next_hop)
                if not current: 
                    break
                
                if current.uplink_bw > 0:
                    gateway_loads[current.node_id] += node.v_ruch
                    break
                    
    # 2. Aplikujemy prawa fizyki dla bramek na podstawie ich obciążenia
    for g_id, load in gateway_loads.items():
        gateway = nodes[g_id]
        
        # Udajemy stały "obcy ruch" od innych ludzi w internecie (np. 20% łącza)
        obcy_ruch = abs(random.gauss(gateway.uplink_bw * 0.2, gateway.uplink_bw * 0.05))
        
        # Aktualizacja B_wolne (Maksymalne łącze minus nasz ruch minus obcy ruch)
        nowe_b_wolne = gateway.uplink_bw - load - obcy_ruch
        gateway.b_wolne = max(0.1, nowe_b_wolne)
        
        # Bufferbloat: PING rośnie asymptotycznie, gdy łącze się zapycha!
        zajetosc_proc = (load + obcy_ruch) / gateway.uplink_bw
        if zajetosc_proc > 0.99: zajetosc_proc = 0.99
        
        # Wzór matematyczny z teorii kolejek (Ping bazowy 15ms + czas w kolejce)
        gateway.ping_avg = 15.0 + (5.0 / (1.0 - zajetosc_proc))
        gateway.ping_mdev = gateway.ping_avg * random.uniform(0.05, 0.2) # Jitter to 5-20% pingu
        
        print(f" [Środowisko] {g_id}: Obciążenie = {load:.1f} Mbps, B_wolne = {gateway.b_wolne:.1f} Mbps, Ping = {gateway.ping_avg:.1f} ms")


# --- URUCHOMIENIE DYNAMICZNEJ SYMULACJI ---
if __name__ == "__main__":
    
    config_file = 'topology.json'
    print(f"Wczytywanie węzłów z pliku: {config_file}...")
    try:
        nodes, base_links = load_topology_from_json(config_file)
    except FileNotFoundError:
        print(f"Błąd: Nie znaleziono pliku {config_file}!")
        exit()

    # Definicja dynamicznych scenariuszy (kroków czasowych)
    scenarios = [
        {
            "step": 1,
            "desc": "Krok 1: Stan normalny (Dobry zasieg, N1 dziala)",
            "wifi_range": 80,
            "node_updates": {
                "N1": {"uplink_bw": 100},
                "N2": {"uplink_bw": 5}
            },
            "link_updates": []
        },
        {
            "step": 2,
            "desc": "Krok 2: Awaria swiatlowodu w N1! Siec musi zmienic bramke.",
            "wifi_range": 80,
            "node_updates": {
                "N1": {"uplink_bw": 0},
                "N2": {"uplink_bw": 5}
            },
            "link_updates": []
        },
        {
            "step": 3,
            "desc": "Krok 3: Pogorszenie warunkow radiowych (Zasieg spada do 45m).",
            "wifi_range": 60,
            "node_updates": {
                "N1": {"uplink_bw": 0},
                "N2": {"uplink_bw": 5}
            },
            "link_updates": [
                {"u": "N2", "v": "N3", "b_phy": 10, "u_eter": 0.6},
                {"u": "N2", "v": "N4", "b_phy": 10, "u_eter": 0.6}
            ]
        },
        {
            "step": 4,
            "desc": "Krok 4: N1 naprawione, powrot do stanu normalnego.",
            "wifi_range": 80,
            "node_updates": {
                "N1": {"uplink_bw": 100},
                "N2": {"uplink_bw": 5}
            },
            "link_updates": []
        }
    ]

    print("\n--- ROZPOCZĘCIE DYNAMICZNEJ SYMULACJI ---")
    
    for scen in scenarios:
        print(f"\nUruchamianie: {scen['desc']}")
        
        # 1. Zastosowanie zmieniajacych sie warunkow (wezly i linki)
        for node_id, updates in scen.get('node_updates', {}).items():
            node = nodes.get(node_id)
            if not node:
                continue
            node.update_gateway_metrics(
                uplink_bw=updates.get('uplink_bw'),
                c_tech=updates.get('c_tech'),
                ping_avg=updates.get('ping_avg'),
                ping_mdev=updates.get('ping_mdev'),
                v_ruch=updates.get('v_ruch'),
                b_wolne=updates.get('b_wolne'),
            )
        current_range = scen.get('wifi_range', 80)
        
        # 2. Reset stanu węzłów (czyszczenie starych tras)
        for node in nodes.values():
            node.reset_state()
            
        # 3. Przeliczenie polaczen (albo z JSON, albo auto zakres)
        link_updates = scen.get('link_updates', [])
        if base_links:
            edges_config = build_edges_from_links(base_links, link_updates)
        else:
            edges_config = auto_generate_edges(
                nodes,
                max_range_meters=current_range,
                default_u_eter=scen.get('default_u_eter', DEFAULT_U_ETER),
            )
            edges_config = apply_link_updates(edges_config, link_updates)
        
        for edge in edges_config:
            u = edge['u']
            v = edge['v']
            b_phy = edge.get('b_phy', DEFAULT_B_PHY)
            u_eter = edge.get('u_eter', DEFAULT_U_ETER)
            if u in nodes and v in nodes:
                nodes[u].add_neighbor(v, b_phy, u_eter)
                nodes[v].add_neighbor(u, b_phy, u_eter)
                
        # 4. Uruchomienie algorytmu (Self-Healing)
        # Przebieg 1: Znalezienie tras na czysto
        run_mesh_routing_algorithm(nodes)
        
        # Symulacja upływu czasu: sieć zaczyna nadawać i środowisko reaguje
        update_environment_physics(nodes)
        
        # Przebieg 2: Reakcja węzłów na nowy PING i spadek B_wolne (Self-Healing)
        run_mesh_routing_algorithm(nodes)
        
        # 5. Wygenerowanie i zapis klatki symulacji
        filename = f"symulacja_krok_{scen['step']}.png"
        link_loads = calculate_link_loads(nodes)
        visualize_network(
            nodes,
            edges_config,
            title=scen['desc'],
            filename=filename,
            link_loads=link_loads,
        )

    print("\nSymulacja zakończona. Wygenerowano serię obrazów PNG pokazujących adaptację sieci.")