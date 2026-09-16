"""Object asset library: `assets/<object_id>/asset.json`.

One object is described once -- geometry, physics and provenance -- so several tasks can
reference it by id instead of each inlining its own copy. Nothing consumes the library yet:
this module only resolves, loads and validates it.

An absent section means "not established", not "use a default". A consumer that needs
`physics.mass_kg` has to refuse an object without it rather than invent a number.
"""
import pathlib,re
from .contracts import load,sha

# Roles, kinds, confidence levels and the other value domains live in the schema file alone;
# a second list here would drift from it. These two are checked in code because they also run
# with check_schema=False and because they involve the filesystem.
MESH_SUFFIXES=['.glb','.gltf']
ID_PATTERN=re.compile(r'^[A-Za-z][A-Za-z0-9_.-]*$')

def library_root():
 """The committed library at the repository root; derived, so no environment variable is needed.

 The library belongs to the checkout, not to the installed package: a wheel has no `assets/`,
 and `validate_library` reports that rather than inventing a path.
 """
 return pathlib.Path(__file__).resolve().parents[2]/'assets'

def object_dir(root,object_id):
 return pathlib.Path(root)/object_id

def validate_asset(doc,directory=None,check_schema=True):
 """Validate one asset document; `directory` pins the id and resolves a mesh path."""
 if check_schema:
  from jsonschema import Draft202012Validator
  schema=load(pathlib.Path(__file__).parent/'schemas/asset.schema.json');Draft202012Validator(schema).validate(doc)
 if not ID_PATTERN.match(doc['id']):raise ValueError('Invalid asset id: '+str(doc['id']))
 if directory is not None and pathlib.Path(directory).name!=doc['id']:raise ValueError('%s: id does not match its directory %s'%(doc['id'],pathlib.Path(directory).name))
 g=doc['geometry']
 if g['kind']=='box' and min(g['size_m'])<=0:raise ValueError(doc['id']+': non-positive size_m')
 # Passing the object directory means "this is the asset's home", so its mesh must ship with it.
 if g['kind']=='mesh':mesh_path(doc,directory,must_exist=directory is not None)
 if 'physics' in doc:
  p=doc['physics']
  if p['mass_kg']<=0:raise ValueError(doc['id']+': non-positive mass_kg')
  if p['friction']<0:raise ValueError(doc['id']+': negative friction')
  if 'inertia_diagonal_kg_m2' in p and min(p['inertia_diagonal_kg_m2'])<=0:raise ValueError(doc['id']+': non-positive inertia')
 if 'visual' in doc:
  v=doc['visual']
  if 'color_linear' in v and not all(0<=x<=1 for x in v['color_linear']):raise ValueError(doc['id']+': albedo outside [0,1]')
  if not 0<=v.get('roughness',.5)<=1:raise ValueError(doc['id']+': invalid roughness')
 return doc

def mesh_path(doc,directory,must_exist=False):
 """Resolve a mesh asset's GLB/GLTF, refusing any path that leaves its own object directory."""
 if directory is None:raise ValueError(doc['id']+': mesh assets need the object directory to resolve geometry.path')
 directory=pathlib.Path(directory).resolve();relative=doc['geometry']['path']
 if pathlib.Path(relative).suffix.lower() not in MESH_SUFFIXES:raise ValueError(doc['id']+': geometry.path must be GLB/GLTF, got '+relative)
 source=directory/relative
 if source.is_symlink():raise ValueError(doc['id']+': geometry.path is a symlink, which the library rejects')
 source=source.resolve()
 if not source.is_relative_to(directory):raise ValueError(doc['id']+': geometry.path escapes the object directory')
 if must_exist and not source.is_file():raise ValueError(doc['id']+': missing mesh file '+relative)
 return source

def load_asset(path,directory=None,check_schema=True):
 """Read one asset.json, validate it, and return the document."""
 path=pathlib.Path(path);doc=load(path);validate_asset(doc,path.resolve().parent if directory is None else directory,check_schema);return doc

def describe_error(error):
 """One readable line for a report: jsonschema's str() dumps the whole schema."""
 message=getattr(error,'message',None);path=getattr(error,'json_path',None)
 return '%s (at %s)'%(message,path) if message and path else str(error)

def validate_library(root=None):
 """Validate every object in the library. Reports a bad asset instead of raising on it."""
 root=library_root() if root is None else pathlib.Path(root)
 if not root.is_dir():return {'passed':False,'root':str(root),'objects':[],'errors':['Library root is missing: '+str(root)]}
 objects=[];errors=[]
 for directory in sorted(p for p in root.iterdir() if p.is_dir()):
  if directory.is_symlink():errors.append(directory.name+': symlinked object directory');continue
  spec=directory/'asset.json'
  if not spec.is_file():errors.append(directory.name+': missing asset.json');continue
  try:
   doc=load_asset(spec,directory);row={'id':doc['id'],'role':doc['role'],'kind':doc['geometry']['kind'],'mesh':None,'sha256':None}
   if row['kind']=='mesh':
    source=mesh_path(doc,directory,must_exist=True);row['mesh']=str(source.relative_to(root));row['sha256']=sha(source)
   objects.append(row)
  except Exception as error:
   message=describe_error(error)
   # Our own ValueErrors already name the object; jsonschema's do not.
   errors.append(message if message.startswith(directory.name+':') else '%s: %s'%(directory.name,message))
 return {'passed':not errors,'root':str(root),'objects':objects,'errors':errors}
