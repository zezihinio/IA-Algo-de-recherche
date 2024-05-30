import json
import random
import time
import sys
import os
import datetime
import heapq
import statistics
from collections import deque


def write_stats_to_file(filename, data):
    # Vérifier si le fichier existe, sinon le créer
    if not os.path.exists(filename):
        with open(filename, 'w') as file:
            file.write("")  # Créer le fichier vide s'il n'existe pas

    # Écrire les données dans le fichier
    with open(filename, 'a') as file:
        file.write(data)

def charger_donnees_metro():
    chemin_absolu = os.path.join(os.path.dirname(__file__), 'data', 'metro_data.json')
    with open(chemin_absolu, 'r', encoding='utf-8') as fichier:
        return json.load(fichier)

def heuristic(current_station, goal_station, metro_data):
    """ Calculer l'heuristique basée sur le nombre minimal de stations restantes. """
    for line in metro_data['lignes']:
        if 'stations' in line or 'branches' in line:
            stations = line['stations'] if 'stations' in line else [station for branch in line['branches'] for station in branch['stations']]
            if current_station in [station['nom'] for station in stations] and goal_station in [station['nom'] for station in stations]:
                start_index = next((i for i, station in enumerate(stations) if station['nom'] == current_station), None)
                end_index = next((i for i, station in enumerate(stations) if station['nom'] == goal_station), None)
                return abs(start_index - end_index) if start_index is not None and end_index is not None else sys.maxsize
    return sys.maxsize

def a_star_search2(metro_data, start, goal):
    """
    L'algorithme A* est une recherche efficace qui combine les avantages de la recherche en largeur (BFS) et
     de la recherche guidée par heuristique (Greedy Search). A* utilise une fonction de coût pour explorer
     les chemins de manière à trouver le chemin le plus court de manière optimale.
    :param metro_data:
    :param start:
    :param goal:
    :return:
    """
    start_time = time.time()
    queue = []

    #La queue est implémentée comme une heap pour toujours extraire le chemin avec le  plus petit coût total (f_score).
    # La heap est initialisée avec le chemin ne contenant que la station de départ, un g_score de 0, et
    # un f_score égal à l'heuristique estimée.
    heapq.heappush(queue, (0 + heuristic(start, goal, metro_data), [(start, None)], 0)) # Pousser le chemin initial avec son coût
    visited = set() # Ensemble pour garder une trace des stations visitées

    iterations = 0

    while queue:
        iterations += 1
        f_score, path, g_score = heapq.heappop(queue) # Retirer le chemin avec le plus petit f_score de la heap
        current_station, current_line = path[-1] # Dernière station et ligne du chemin actuel

        if current_station == goal: # Vérifier si la station actuelle est la station d'arrivée
            lines_used = summarize_lines_used(path) # toutes les lignes utilisées dans le chemin
            return {
                'path': [station for station, line in path],
                'lines': lines_used,
                'iterations': iterations,
                'execution_time': time.time() - start_time,
                'memory_usage': sys.getsizeof(queue),
                'travel_time': g_score,
                'path_length': len(path)
            }
        # Si la station actuelle a déjà été visitée, passer..
        if current_station in visited:
            continue
        # Marquer la station actuelle comme visitée
        visited.add(current_station)

        # Expansion des successeurs
        for line in metro_data['lignes']:
            #On récupère toutes les stations de chaque ligne, y compris les stations des branches.
            stations = line['stations'] if 'stations' in line else []
            for branch in line.get('branches', []):
                stations += branch['stations']
            # On vérifie si la station courante fait partie des stations de la ligne.
            if current_station in [s['nom'] for s in stations]:
                #On trouve les stations adjacentes (précédente et suivante) et les ajoute à la liste des voisins.
                idx = next((i for i, s in enumerate(stations) if s['nom'] == current_station), None)
                if idx is not None:
                    neighbors = []
                    if idx > 0:
                        neighbors.append((stations[idx - 1]['nom'], line['id']))
                    if idx < len(stations) - 1:
                        neighbors.append((stations[idx + 1]['nom'], line['id']))
                    for neighbor, neighbor_line in neighbors:
                        # On vérifie que la station suivante n'est pas déjà dans le chemin pour éviter les boucles
                        if neighbor not in [s for s, l in path]:
                            extra_time = 1 if neighbor_line == current_line else 2
                            new_path = path + [(neighbor, neighbor_line)]
                            new_g_score = g_score + extra_time
                            new_f_score = new_g_score + heuristic(neighbor, goal, metro_data)
                            #On ajoute le nouveau chemin, les scores g_score et f_score à la queue
                            heapq.heappush(queue, (new_f_score, new_path, new_g_score))

    return {'path': None, 'iterations': iterations, 'execution_time': time.time() - start_time, 'memory_usage':
        sys.getsizeof(queue), 'travel_time': None, 'path_length': 0}


def beam_search2(metro_data, start, goal, width=2):
    """variation de BFS qui limite le nombre de chemins explorés à chaque niveau. Cela permet de
    réduire la complexité en temps et en mémoire en ne conservant que les chemins les plus
    prometteurs selon une heuristique. """
    start_time = time.time()
    queue = [([(start, None)], 0, 0)]  # Chemin initial avec station, ligne, temps de trajet, et heuristique
    iterations = 0

    while queue:
        iterations += 1
        new_queue = []
        for path, travel_time, _ in queue:
            current_station, current_line = path[-1]  # Dernière station et ligne du chemin actuel
            if current_station == goal:  # Vérifier si la station actuelle est la station d'arrivée
                lines_used = summarize_lines_used(path)  # Résumer les lignes utilisées dans le chemin
                return {
                    'path': [station for station, line in path],  # Retourner le chemin trouvé
                    'lines': lines_used,  # Retourner les lignes utilisées
                    'iterations': iterations,  # Retourner le nombre d'itérations
                    'execution_time': time.time() - start_time,  # Retourner le temps total d'exécution
                    'memory_usage': sys.getsizeof(queue),  # Retourner l'utilisation de la mémoire par la queue
                    'travel_time': travel_time,  # Retourner le temps total de trajet
                    'path_length': len(path)  # Retourner la longueur du chemin
                }

            # Exploration des stations voisines
            for line in metro_data['lignes']:
                stations = line.get('stations', []) + [station for branch in line.get('branches', []) for station in branch['stations']]
                #On vérifie si la station courante fait partie des stations de la ligne.
                if current_station in [s['nom'] for s in stations]:
                    # On obtient l'index de la station courante dans la liste des stations.
                    idx = next((i for i, s in enumerate(stations) if s['nom'] == current_station), None)
                    if idx is not None:
                        # Si l'index est trouvé, on ajoute les stations adjacentes à la nouvelle queue en utilisant
                        # la fonction add_adjacent_stations_beam
                        add_adjacent_stations_beam(new_queue, path, travel_time, line['id'], idx, stations, goal, metro_data)
        # On ne conserve que les width meilleurs chemins, limitant ainsi le nombre de chemins explorés
        # à chaque itération basés sur l'heuristique
        queue = sorted(new_queue, key=lambda x: x[2])[:width]
    # Return Si Aucun Chemin n'est Trouvé
    return {'path': None, 'iterations': iterations, 'execution_time': time.time() - start_time, 'memory_usage': sys.getsizeof(queue), 'travel_time': None, 'path_length': 0}

def add_adjacent_stations_beam(new_queue, path, travel_time, line_id, idx, stations, goal, metro_data):
    """
    ajoute les stations adjacentes à la nouvelle queue pour Beam Search.
    1.Récupérer la Dernière Station et Ligne: On récupère la dernière station et la dernière ligne du chemin actuel.
    2.Lister les Stations Adjacentes: On liste les stations adjacentes (précédente et suivante).
    3.Vérifier les Boucles: On vérifie que la station suivante n'est pas déjà dans le chemin pour éviter les boucles.
    4.Créer un Nouveau Chemin: On crée un nouveau chemin en ajoutant la station suivante.
    5.Calculer le Temps de Trajet: On met à jour le temps de trajet en ajoutant un temps supplémentaire si la ligne change.
    6.Calculer l'Heuristique: On calcule l'heuristique pour la nouvelle station.
    7.Ajouter à la Nouvelle Queue: On ajoute le nouveau chemin, le temps de trajet et l'heuristique à la nouvelle queue.
    """
    last_station, last_line = path[-1]
    adjacent_stations = []
    if idx > 0:
        adjacent_stations.append((stations[idx - 1]['nom'], line_id))
    if idx < len(stations) - 1:
        adjacent_stations.append((stations[idx + 1]['nom'], line_id))
    for next_station, next_line in adjacent_stations:
        if next_station not in [station for station, line in path]:
            new_path = path + [(next_station, next_line)]
            extra_time = 1 if next_line == last_line else 2
            new_travel_time = travel_time + extra_time
            new_heuristic = heuristic(next_station, goal, metro_data)
            new_queue.append((new_path, new_travel_time, new_heuristic))

def summarize_lines_used(path):
    """résume les lignes utilisées dans le chemin trouvé"""
    lines_used = []
    last_line = None
    line_start_station = path[0][0]

    for i, (station, line) in enumerate(path):
        if line != last_line:
            if last_line is not None:
                lines_used.append(f"{last_line} {line_start_station} - {path[i - 1][0]}")
            line_start_station = station
            last_line = line

    if last_line is not None:
        lines_used.append(f"{last_line} {line_start_station} - {path[-1][0]}")

    return lines_used

def greedy_search2(metro_data, start, goal):
    """
    Utilise une heuristique pour guider la recherche vers l'objectif (la station d'arrivée) en sélectionnant à chaque
    étape le chemin qui semble le plus prometteur en termes de distance estimée jusqu'à l'objectif.
    """
    start_time = time.time()  # Démarrer le chronomètre
    queue = [([(start, None)], 0, 0)]  # Chemin initial, temps de trajet, et heuristique
    visited = set()  # Ensemble pour garder une trace des stations visitées
    iterations = 0  # Compteur d'itérations

    while queue:
        iterations += 1  # Incrémente le compteur à chaque itération de la boucle
        queue.sort(key=lambda x: x[2])  # Trier la queue en fonction de l'heuristique
        path, travel_time, _ = queue.pop(0)  # Retirer le chemin avec la meilleure heuristique

        # Verifier si objéctif atteint ?
        current_station, current_line = path[-1]  # Dernière station et ligne du chemin actuel
        if current_station == goal:  # Vérifier si la station actuelle est la station d'arrivée
            end_time = time.time()  # Arrêter le chronomètre
            lines_used = summarize_lines_used(path)  # Résumer les lignes utilisées dans le chemin
            return {
                'path': [station for station, line in path],  # Retourner le chemin trouvé
                'lines': lines_used,  # Retourner les lignes utilisées
                'iterations': iterations,  # Retourner le nombre d'itérations
                'execution_time': end_time - start_time,  # Retourner le temps total d'exécution
                'memory_usage': sys.getsizeof(queue),  # Retourner l'utilisation de la mémoire par la queue
                'travel_time': travel_time,  # Retourner le temps total de trajet
                'path_length': len(path)  # Retourner la longueur du chemin
            }

        # Vérifier si la station actuelle avec la ligne n'a pas été visitée
        if (current_station, current_line) not in visited:
            visited.add((current_station, current_line))  # Marquer la station actuelle comme visitée

            # Explorer les Lignes de Métro et Ajouter les Stations Adjacentes
            for line in metro_data['lignes']:  # Parcourir toutes les lignes de métro
                stations = line.get('stations', []) + [station for branch in line.get('branches', []) for station in branch['stations']]
                if current_station in [s['nom'] for s in stations]:  # Vérifier si la station actuelle est dans les stations de la ligne
                    idx = next((i for i, s in enumerate(stations) if s['nom'] == current_station), None)
                    if idx is not None:
                        add_adjacent_stations(queue, path, travel_time, line['id'], idx, stations, goal, metro_data)  # Ajouter les stations adjacentes à la queue

    return {'path': None, 'iterations': iterations, 'execution_time': time.time() - start_time,
            'memory_usage': sys.getsizeof(queue), 'travel_time': None, 'path_length': 0}


def add_adjacent_stations(queue, path, travel_time, line_id, idx, stations, goal, metro_data):
    """
    Ajouter des stations adjacentes si elles ne sont pas déjà dans le chemin pour éviter les boucles
    :param queue:
    :param path:
    :param travel_time:
    :param line_id:
    :param idx:
    :param stations:
    :param goal:
    :param metro_data:
    :return:
    """
    last_station, last_line = path[-1]
    adjacent_stations = []
    if idx > 0:
        adjacent_stations.append((stations[idx - 1]['nom'], line_id))
    if idx < len(stations) - 1:
        adjacent_stations.append((stations[idx + 1]['nom'], line_id))
    for next_station, next_line in adjacent_stations:
        if next_station not in [station for station, line in path]: # Éviter les boucles
            new_path = path + [(next_station, next_line)]  # Nouveau chemin avec la station suivante
            extra_time = 1 if next_line == last_line else 2
            new_travel_time = travel_time + extra_time # Mise à jour du temps de trajet
            new_heuristic = heuristic(next_station, goal, metro_data) # Calcul de l'heuristique pour la nouvelle station
            queue.append((new_path, new_travel_time, new_heuristic))



def random_search2(metro_data, start, goal):
    start_time = time.time()  # Démarrer le chronomètre
    queue = [([(start, None)], 0)]  # Queue initiale avec le chemin et le temps de trajet initialisé à 0
    iterations = 0  # Compteur d'itérations

    while queue:
        iterations += 1
        path_index = random.randint(0, len(queue) - 1)
        path, travel_time = queue.pop(path_index)  # Récupérer un chemin au hasard et son temps de trajet

        current_station, current_line = path[-1]
        if current_station == goal:
            lines_used = summarize_lines_used(path)
            return {
                'path': [station for station, line in path],
                'lines': lines_used,
                'iterations': iterations,
                'execution_time': time.time() - start_time,
                'memory_usage': sys.getsizeof(queue),
                'travel_time': travel_time,
                'path_length': len(path)
            }

        # Exploration des stations voisines
        for line in metro_data['lignes']:
            stations = line.get('stations', []) + [station for branch in line.get('branches', []) for station in branch['stations']]
            if current_station in [s['nom'] for s in stations]:
                idx = next((i for i, s in enumerate(stations) if s['nom'] == current_station), None)
                if idx is not None:
                    explore_neighbors_random(path, travel_time, line['id'], idx, stations, queue)

    return {
        'path': None,
        'iterations': iterations,
        'execution_time': time.time() - start_time,
        'memory_usage': sys.getsizeof(queue),
        'travel_time': None,
        'path_length': 0
    }

def explore_neighbors_random(path, travel_time, line_id, idx, stations, queue):
    last_station, last_line = path[-1]
    adjacent_stations = []
    if idx > 0:
        adjacent_stations.append((stations[idx - 1]['nom'], line_id))
    if idx < len(stations) - 1:
        adjacent_stations.append((stations[idx + 1]['nom'], line_id))
    for next_station, next_line in adjacent_stations:
        if next_station not in [station for station, line in path]:  # Éviter les boucles
            extra_time = 1 if next_line == last_line else 2
            new_path = path + [(next_station, next_line)]
            new_travel_time = travel_time + extra_time
            queue.insert(random.randint(0, len(queue)), (new_path, new_travel_time))

def bfs_search2(metro_data, depart, arrivee):
    """
        Breadth-First Search (BFS) algorithm for finding the shortest path in a metro system.

        Parameters:
        - metro_data (dict): The metro system data containing lines and stations.
        - depart (str): The starting station.
        - arrivee (str): The destination station.

        Returns:
        - dict: A dictionary containing the following keys:
            - 'path': List of stations in the found path.
            - 'lines': List of line segments used in the path.
            - 'iterations': Number of iterations the algorithm ran.
            - 'execution_time': Total execution time of the algorithm.
            - 'memory_usage': Memory usage by the queue during execution.
            - 'travel_time': Total travel time of the path.
            - 'path_length': Number of stations in the found path.
        """
    start_time = time.time()  # Démarrer le chronomètre
    queue = deque([([(depart, None)], 0)])  # Queue initiale avec le chemin et ligne
    visited = set()  # Ensemble pour garder une trace des stations visitées avec la ligne
    iterations = 0  # Compteur d'itérations

    while queue:
        iterations += 1
        path, travel_time = queue.popleft()  # Retirer le premier chemin de la file
        current_station, current_line = path[-1]  # Dernière station et ligne du chemin actuel

        if current_station == arrivee:
            lines_used = []
            previous_line = None
            line_start_station = path[0][0]

            for station, line in path:
                if line != previous_line:
                    if previous_line is not None:
                        lines_used.append(f"{previous_line} {line_start_station} - {previous_station}")
                    line_start_station = station
                    previous_line = line
                previous_station = station

            if previous_line is not None:
                lines_used.append(f"{previous_line} {line_start_station} - {previous_station}")

            return {
                'path': [station for station, line in path],
                'lines': lines_used,
                'iterations': iterations,
                'execution_time': time.time() - start_time,
                'memory_usage': sys.getsizeof(queue),
                'travel_time': travel_time,
                'path_length': len(path)
            }

        # Marquer la station actuelle comme visitée
        if (current_station, current_line) not in visited:
            visited.add((current_station, current_line))

            # Explorer les stations voisines
            for line in metro_data['lignes']:
                stations = line.get('stations', [])
                if 'branches' in line:
                    for branch in line['branches']:
                        stations.extend(branch['stations'])

                # Vérifier si la station actuelle est dans la liste des stations pour cette ligne
                if current_station in [s['nom'] for s in stations]:
                    idx = next((i for i, s in enumerate(stations) if s['nom'] == current_station), None)
                    if idx is not None:
                        # Ajouter la station précédente à la file si elle n'a pas été visitée
                        if idx > 0:
                            prev_station = stations[idx - 1]['nom']
                            if (prev_station, line['id']) not in visited:
                                queue.append((path + [(prev_station, line['id'])],
                                              travel_time + 1 + (1 if current_line != line['id'] else 0)))
                        # Ajouter la station suivante à la file si elle n'a pas été visitée
                        if idx < len(stations) - 1:
                            next_station = stations[idx + 1]['nom']
                            if (next_station, line['id']) not in visited:
                                queue.append((path + [(next_station, line['id'])],
                                              travel_time + 1 + (1 if current_line != line['id'] else 0)))

    return {'path': None, 'iterations': iterations, 'execution_time': time.time() - start_time,
            'memory_usage': sys.getsizeof(queue), 'travel_time': None, 'path_length': 0}

def execute_algorithm(algorithm, name, metro_data, depart, arrivee):
    log_data = f"\nDépart : {depart}\nArrivée : {arrivee}\nAlgorithme : {name}\n"
    print("algorithme : ", name)
    results = algorithm(metro_data, depart, arrivee)
    if results and results['path']:
        path_str = " -> ".join(results['path'])
        log_data += f"Chemin trouvé par {name} :\n{path_str}\n"
        log_data += f"Lignes utilisé : {results['lines']} \n"
        log_data += f"Nombre d'itérations : {results['iterations']}\n"
        log_data += f"Temps d'exécution : {results['execution_time']:.2f} secondes\n"
        log_data += f"Utilisation de la mémoire : {results['memory_usage']} bytes\n"
        log_data += f"Longueur du chemin : {results['path_length']} stations\n"
        log_data += f"temps de voyage : {results['travel_time']} minutes\n"

        print(f"Chemin trouvé par {name} :\n{path_str}\n")
        print(f"Lignes utilisé : {results['lines']} \n")
        print(f"Nombre d'itérations : {results['iterations']}\n")
        print(f"Temps d'exécution : {results['execution_time']:.2f} secondes\n")
        print(f"Utilisation de la mémoire : {results['memory_usage']} bytes\n")
        print(f"Longueur du chemin : {results['path_length']} stations\n")
        print(f"temps de voyage : {results['travel_time']} minutes\n")
    else:
        log_data += f"Aucun chemin trouvé par {name}.\n"
    return log_data

def execute_random_searches(metro_data, depart, arrivee):
    log_data = f"\nDépart : {depart}\nArrivée : {arrivee}\nAlgorithme : Random Search (5 exécutions)\n"
    random_search_times = []
    random_search_memory = []

    print("algorithme : Random Search")
    for i in range(5):
        results = random_search2(metro_data, depart, arrivee)
        if results and results['path']:
            path_str = " -> ".join(results['path'])
            log_data += f"Exécution {i+1}:\nChemin trouvé par Random Search :\n{path_str}\n"
            log_data += f"Lignes utilisé : {results['lines']} \n"
            log_data += f"Nombre d'itérations : {results['iterations']}\n"
            log_data += f"Temps d'exécution : {results['execution_time']:.2f} secondes\n"
            log_data += f"Utilisation de la mémoire : {results['memory_usage']} bytes\n"
            log_data += f"Longueur du chemin : {results['path_length']} stations\n"
            log_data += f"temps de voyage : {results['travel_time']} minutes\n"

            print(f"Exécution {i+1}:\nChemin trouvé par Random Search :\n{path_str}\n")
            print(f"Lignes utilisé : {results['lines']} \n")
            print(f"Nombre d'itérations : {results['iterations']}\n")
            print(f"Temps d'exécution : {results['execution_time']:.2f} secondes\n")
            print(f"Utilisation de la mémoire : {results['memory_usage']} bytes\n")
            print(f"Longueur du chemin : {results['path_length']} stations\n")
            print(f"temps de voyage : {results['travel_time']} minutes\n")
            print("================================")

            random_search_times.append(results['execution_time'])
            random_search_memory.append(results['memory_usage'])
        else:
            log_data += f"Exécution {i+1} : Aucun chemin trouvé.\n"
        time.sleep(1)

    # Calcul des moyennes
    avg_time = statistics.mean(random_search_times)
    avg_memory = statistics.mean(random_search_memory)

    log_data += f"Moyennes sur 5 exécutions de Random Search :\n"
    log_data += f"Temps d'exécution moyen : {avg_time:.2f} secondes\n"
    log_data += f"Utilisation de la mémoire moyenne : {avg_memory} bytes\n"

    print("================================")
    print(f"Moyennes sur 5 exécutions de Random Search :\n")
    print(f"Temps d'exécution moyen : {avg_time:.2f} secondes\n")
    print(f"Utilisation de la mémoire moyenne : {avg_memory} bytes\n")
    print("================================")

    return log_data

def main():
    metro_data = charger_donnees_metro()
    now = datetime.datetime.now().strftime("%d-%B %H:%M")
    log_data = f"{now}:\n"

    print("Test de 5 algorithmes de recherche différents :")

    # Définir les stations de départ et d'arrivée
    depart = "Alexandre Dumas"
    arrivee = "Balard"

    # Liste des algorithmes à exécuter
    algorithms = {
        'BFS': bfs_search2,
        'Greedy Search': greedy_search2,
        'Beam Search': beam_search2,
        'A* Search': a_star_search2,
    }

    # Exécuter chaque algorithme et recueillir les résultats
    for name, algorithm in algorithms.items():
        log_data += execute_algorithm(algorithm, name, metro_data, depart, arrivee)
        time.sleep(1)  # Pause d'une seconde entre chaque algorithme

    # Exécution de Random Search 5 fois et calcul des moyennes
    log_data += execute_random_searches(metro_data, depart, arrivee)

    log_data += "================================\n"
    write_stats_to_file("stat.txt", log_data)

if __name__ == '__main__':
    main()

