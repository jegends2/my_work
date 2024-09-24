# transform_to_utm.py
import fields2cover as f2c

def transform_to_utm(field):
    print("Transforming field to UTM...")
    f2c.Transform.transformToUTM(field)
    print("Transformation to UTM complete.")

if __name__ == "__main__":
    from import_field import import_field
    file_path = '/home/adsol/costmap.xml'
    field = import_field(file_path)
    transform_to_utm(field)
