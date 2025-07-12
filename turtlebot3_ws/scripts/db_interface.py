import psycopg2

# DB connection config
DB_CONFIG = {
    'dbname': 'map_db',
    'user': 'postgres',
    'password': 'password',
    'host': 'localhost',
    'port': 5432
}

def get_connection():
    return psycopg2.connect(**DB_CONFIG)

def get_wormhole_data(from_room, to_room):
    """Returns a dict of all fields for the given room pair."""
    try:
        conn = get_connection()
        cur = conn.cursor()
        query = """
            SELECT * FROM multimap
            WHERE from_room = %s AND to_room = %s
        """
        cur.execute(query, (from_room, to_room))
        result = cur.fetchone()
        cur.close()
        conn.close()

        if result:
            return {
                'from_room': result[0],
                'to_room': result[1],
                'from_pose_x': result[2],
                'from_pose_y': result[3],
                'from_yaw': result[4],
                'to_pose_x': result[5],
                'to_pose_y': result[6],
                'to_yaw': result[7],
            }
        else:
            return None
    except Exception as e:
        print(f"Error: {e}")
        return None

def get_all_pairs():
    """Returns a list of (from_room, to_room) pairs."""
    try:
        conn = get_connection()
        cur = conn.cursor()
        cur.execute("SELECT from_room, to_room FROM multimap")
        pairs = cur.fetchall()
        cur.close()
        conn.close()
        return pairs
    except Exception as e:
        print(f"Error: {e}")
        return []

# === Example Usage ===
if __name__ == '__main__':
    print("All from → to pairs:")
    print(get_all_pairs())

    print("\nDetails for ('room1', 'room2'):")
    details = get_wormhole_data('room1', 'room2')
    print(details if details else "Mapping not found.")
