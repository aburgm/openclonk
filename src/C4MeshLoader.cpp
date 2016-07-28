#include <C4Include.h>
#include <c4group/C4Components.h>
#include <lib/StdMeshLoader.h>
#include <sstream>

bool EraseItemSafe(const char *szFilename) {return false;}

namespace
{

template<typename... T>
void write_object(std::ostream& out, T... t);

template<typename ContainerT>
void write_list(std::ostream& out, const ContainerT& cont);

template<typename ContainerT>
void write_ptr_list(std::ostream& out, const ContainerT& cont);

template<typename T>
void write_entity(std::ostream& out, const std::vector<T*>& vector)
{
	write_ptr_list(out, vector);
}

template<typename T>
void write_entity(std::ostream& out, const std::vector<T>& vector)
{
	write_list(out, vector);
}

void write_entity(std::ostream& out, int i)
{
	out << i;
}

void write_entity(std::ostream& out, unsigned int i)
{
	out << i;
}

void write_entity(std::ostream& out, float f)
{
	// TODO: C locale?
	out << f;
}

void write_entity(std::ostream& out, const char* s)
{
	out << "\"" << s << "\"";
}

void write_entity(std::ostream& out, const std::string& s)
{
	write_entity(out, s.c_str());
}

void write_entity(std::ostream& out, const StdMeshBox& box)
{
	out << "[" << box.x1 << ", " << box.x2 << ", " << box.y1 << ", " << box.y2 << ", " << box.z1 << ", " << box.z2 << "]";
}

void write_entity(std::ostream& out, const StdMesh::Vertex& vtx)
{
	const std::vector<float> pos { vtx.x, vtx.y, vtx.z };
	const std::vector<float> normal { vtx.nx, vtx.ny, vtx.nz };
	const std::vector<float> texcoord { vtx.u, vtx.v };
	std::vector<float> bone_weights;
	std::vector<unsigned int> bone_indices;
	for (unsigned int i = 0; i < StdMeshVertex::MaxBoneWeightCount; ++i)
	{
		if (vtx.bone_weight[i] > 0.0f)
		{
			bone_weights.push_back(vtx.bone_weight[i]);
			bone_indices.push_back(vtx.bone_index[i]);
		}
	}

	write_object(
		out,
		"pos", &pos,
		"normal", &normal,
		"texcoord", &texcoord,
		"bone-weights", &bone_weights,
		"bone-indices", &bone_indices);
}

void write_entity(std::ostream& out, const StdMeshFace& face)
{
	write_list(out, face.Vertices);
}

void write_entity(std::ostream& out, const StdSubMesh& submesh)
{
	const std::vector<StdMesh::Vertex>* vertices = nullptr;
	if (!submesh.GetVertices().empty())
		vertices = &submesh.GetVertices();

	std::vector<StdMeshFace> faces(submesh.GetNumFaces());
	for (unsigned int i = 0; i < submesh.GetNumFaces(); ++i)
		faces[i] = submesh.GetFace(i);

	const std::string material = submesh.GetMaterial().Name.getData();

	write_object(
		out,
		"vertices", vertices,
		"faces", &faces,
		"material", &material
	);
}

void write_entity(std::ostream& out, const StdMeshTransformation& trans)
{
	std::vector<float> pos { trans.translate.x, trans.translate.y, trans.translate.z };
	std::vector<float> rot { trans.rotate.w, trans.rotate.x, trans.rotate.y, trans.rotate.z };
	std::vector<float> scale { trans.scale.x, trans.scale.y, trans.scale.z };
	write_object(out, "pos", &pos, "rot", &rot, "scale", &scale);
}

struct StdMeshIndexedTrack
{
	const StdMeshTrack* track;
	unsigned int bone_index;
};

struct StdMeshIndexedFrame
{
	const StdMeshKeyFrame* frame;
	float position;
};

void write_entity(std::ostream& out, const StdMeshIndexedFrame& frame)
{
	write_object(out, "position", &frame.position, "transformation", &frame.frame->Transformation);
}

void write_entity(std::ostream& out, const StdMeshIndexedTrack& track)
{
	std::vector<StdMeshIndexedFrame> frames;
	for(const auto& frame: track.track->Frames)
		frames.push_back({&frame.second, frame.first});

	write_object(out, "bone-index", &track.bone_index, "frames", &frames);
}

void write_entity(std::ostream& out, const StdMeshAnimation& anim)
{
	const char* name = anim.Name.getData();
	std::vector<StdMeshIndexedTrack> tracks;
	for (unsigned int i = 0; i < anim.Tracks.size(); ++i)
		if (anim.Tracks[i] != nullptr)
			tracks.push_back({anim.Tracks[i], i});

	write_object(out, "name", &name, "length", &anim.Length, "tracks", &tracks);
}

void write_entity(std::ostream& out, const StdMeshBone& bone)
{
	const char* name = bone.Name.getData();
	unsigned int parent = 0;
	unsigned int* parentPtr = nullptr;
	if (bone.GetParent())
	{
		parent = bone.GetParent()->Index;
		parentPtr = &parent;
	}

	write_object(out, "index", &bone.Index, "id", &bone.ID, "name", &name, "transformation", &bone.Transformation, "parent-index", parentPtr);
}

void write_entity(std::ostream& out, const StdMeshSkeleton& skel)
{
	std::vector<const StdMeshBone*> bones(skel.GetNumBones());
	for (unsigned int i = 0; i < skel.GetNumBones(); ++i)
		bones[i] = &skel.GetBone(i);

	std::vector<const StdMeshAnimation*> animations = skel.GetAnimations();

	write_object(out, "bones", &bones, "animations", &animations);
}

void write_entity(std::ostream& out, const StdMesh& mesh)
{
	const std::vector<StdMesh::Vertex>* shared_geometry = NULL;
	if (!mesh.GetSharedVertices().empty())
		shared_geometry = &mesh.GetSharedVertices();

	float bounding_radius = mesh.GetBoundingRadius();

	std::vector<const StdSubMesh*> submeshes(mesh.GetNumSubMeshes());
	for (unsigned int i = 0; i < mesh.GetNumSubMeshes(); ++i)
		submeshes[i] = &mesh.GetSubMesh(i);

	write_object(
		out,
		"bounding-box", &mesh.GetBoundingBox(),
		"bounding-radius", &bounding_radius,
		"shared-geometry", shared_geometry,
		"submeshes", &submeshes,
		"skeleton", &mesh.GetSkeleton());
}

template<typename T>
bool write_attribs(std::ostream& out, bool first_val, const char* name, const T* val)
{
	if (val)
	{
		if (!first_val)
			out << ", ";
		else
			first_val = false;

		write_entity(out, name);
		out << ": ";
		write_entity(out, *val);
	}

	return first_val;
}

template<typename H, typename... T>
bool write_attribs(std::ostream& out, bool first_val, const char* name, const H* val, T&&... tail)
{
	first_val = write_attribs(out, first_val, name, val);
	first_val = write_attribs(out, first_val, tail...);
	return first_val;
}

template<typename... T>
void write_object(std::ostream& out, T... t)
{
	out << "{";
	write_attribs(out, true, t...);
	out << "}";
}

template<typename ContainerT>
void write_list(std::ostream& out, const ContainerT& cont)
{
	bool first_val = true;
	out << "[";
	for (auto val: cont)
	{
		if (!first_val)
			out << ", ";
		write_entity(out, val);
		first_val = false;
	}
	out << "]";
}

template<typename ContainerT>
void write_ptr_list(std::ostream& out, const ContainerT& cont)
{
	bool first_val = true;
	out << "[";
	for (auto val: cont)
	{
		if (!first_val)
			out << ", ";
		write_entity(out, *val);
		first_val = false;
	}
	out << "]";
}

void load_materials_in_group(C4Group& hGroup)
{
	class DummyLoader: public StdMeshMaterialLoader
	{
	public:
		virtual C4Surface* LoadTexture(const char* name) { return nullptr; }
		virtual StdStrBuf LoadShaderCode(const char* name) { return StdStrBuf(); }
		virtual void AddShaderSlices(C4Shader& shader, int ssc) { }
	};

	DummyLoader loader;
	hGroup.ResetSearch();
	char MaterialFilename[_MAX_PATH + 1]; *MaterialFilename = 0;
	while (hGroup.FindNextEntry(C4CFN_DefMaterials, MaterialFilename, NULL, !!*MaterialFilename))
	{
		StdStrBuf material;
		if (hGroup.LoadEntryString(MaterialFilename, &material))
		{
			try
			{
				StdStrBuf buf;
				buf.Copy(hGroup.GetName());
				buf.Append("/"); buf.Append(MaterialFilename);
				::MeshMaterialManager.Parse(material.getData(), buf.getData(), loader);
			}
			catch (const StdMeshMaterialError& ex)
			{
				LogF("Failed to read material script %s: %s", MaterialFilename, ex.what());
			}
		}
	}
}

void load_skeleton(C4Group &hGroup, const char* szFileName, StdMeshSkeletonLoader& loader)
{
	char* buf = NULL;
	size_t size;

	try
	{
		if (!hGroup.LoadEntry(szFileName, &buf, &size, 1))
			throw std::runtime_error("Cannot load entry");

		if (SEqualNoCase(GetExtension(szFileName), "xml"))
		{
			loader.LoadSkeletonXml(hGroup.GetName(), szFileName, buf, size);
		}
		else
		{
			loader.LoadSkeletonBinary(hGroup.GetName(), szFileName, buf, size);
		}

		delete[] buf;
	}
	catch (const std::runtime_error& ex)
	{
		LogF("Failed to load skeleton in definition %s: %s", hGroup.GetName(), ex.what());
		delete[] buf;
	}
}

void load_skeletons(C4Group& hGroup, StdMeshSkeletonLoader& loader)
{
	// load skeletons
	char Filename[_MAX_PATH+1]; *Filename=0;
	hGroup.ResetSearch();
	while (hGroup.FindNextEntry("*", Filename, NULL, !!*Filename))
	{
		if (!WildcardMatch(C4CFN_DefSkeleton, Filename) && !WildcardMatch(C4CFN_DefSkeletonXml, Filename)) continue;
		load_skeleton(hGroup, Filename, loader);
	}
}

class C4SkeletonManager : public StdMeshSkeletonLoader
{
	virtual StdMeshSkeleton* GetSkeletonByDefinition(const char* definition) const
	{
		LogF("Skeleton loading by definition is not supported");
		return NULL;
	}
};

}

extern "C" char* load_mesh(const char* filename)
{
	StdStrBuf parent;
	GetParentPath(filename, &parent);

	C4Group group;
	if (!group.Open(parent.getData()))
	{
		LogF("Failed to open group %s", parent.getData());
		return NULL;
	}

	::MeshMaterialManager.Clear();
	load_materials_in_group(group);

	C4SkeletonManager skelmgr;
	load_skeletons(group, skelmgr);

	StdBuf mesh_content;
	if (!group.LoadEntry(GetFilename(filename), &mesh_content))
	{
		LogF("Failed to load file %s", filename);
		return NULL;
	}

	std::unique_ptr<StdMesh> mesh;
	try
	{
		mesh.reset(StdMeshLoader::LoadMeshBinary((const char*)mesh_content.getData(), mesh_content.getSize(), ::MeshMaterialManager, skelmgr, group.GetName()));
	}
	catch(const std::exception& ex)
	{
		LogF("%s", ex.what());
		return NULL;
	}

	std::ostringstream stream;
	write_entity(stream, *mesh);

	const std::string str = stream.str();
	char* buf = new char[str.length() + 1];
	std::copy(str.begin(), str.end(), buf);
	buf[str.length()] = '\0';

	return buf;
}

extern "C" void free_mesh(char* text)
{
	delete[] text;
}
