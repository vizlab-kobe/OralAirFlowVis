#pragma once
#include <random>
#include <kvs/OrthoSlice>
#include <kvs/Isosurface>
#include <kvs/PolygonObject>
#include <kvs/PolygonRenderer>
#include <kvs/PolygonImporter>
#include <kvs/UnstructuredVolumeObject>
#include <kvs/StochasticLineRenderer>
#include <kvs/StochasticPolygonRenderer>
#include <kvs/ParticleBasedRenderer>
#include <kvs/CellByCellMetropolisSampling>
#include <kvs/ExternalFaces>
#include <kvs/Bounds>
#include <kvs/String>
#include <kvs/StampTimer>
#include <kvs/StampTimerList>
#include <kvs/Math>
#include <kvs/mpi/StampTimer>
#include <InSituVis/Lib/Adaptor.h>
#include <InSituVis/Lib/Viewpoint.h>
#include <InSituVis/Lib/CubicViewpoint.h>
#include <InSituVis/Lib/SphericalViewpoint.h>
#include <InSituVis/Lib/PolyhedralViewpoint.h>
#include <InSituVis/Lib/StochasticRenderingAdaptor.h>


/*****************************************************************************/
// In-situ visualization settings
/*****************************************************************************/

// Adaptor setting
//----------------------------------------------------------------------------
//#define IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING

// Pipeline setting
//----------------------------------------------------------------------------
//#define IN_SITU_VIS__PIPELINE__ORTHO_SLICE
#define IN_SITU_VIS__PIPELINE__ISOSURFACE
//#define IN_SITU_VIS__PIPELINE__EXTERNAL_FACE
//#define IN_SITU_VIS__PIPELINE__WHOLE_MIN_MAX_VALUES

// Adaptor definition
//----------------------------------------------------------------------------
#if defined( IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING )
namespace { using Adaptor = InSituVis::mpi::StochasticRenderingAdaptor; }
#else
namespace { using Adaptor = InSituVis::mpi::Adaptor; }
#endif

// Parameters
//----------------------------------------------------------------------------
namespace Params
{
struct Output
{
    static const auto Image = true;
    static const auto SubImage = false;
    static const auto SubImageDepth = false;
    static const auto SubImageAlpha = false;
};

const auto ImageSize = kvs::Vec2ui{ 1024, 1024 }; // width x height
const auto AnalysisInterval = 100; // l: analysis (visuaization) time interval

const auto VisibleBoundingBox = false;
const auto VisibleBoundaryMesh = false;

// For IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING
const auto Repeats = 50; // number of repetitions for stochastic rendering
const auto BoundaryMeshOpacity = 30; // opacity value [0-255] of boundary mesh

// Viewpoint settings (Integrated from sample code)
//----------------------------------------------------------------------------
const auto ViewDir = InSituVis::Viewpoint::Direction::Uni; // Uni or Omni

// 座標系はUnstructured gridのデータスケールに合わせて調整が必要な場合がありますが
// サンプルコードの値をそのまま移植しています。
const auto m_min_coord = kvs::Vec3{ -9.0f, -9.0f, -9.0f };
const auto m_max_coord = kvs::Vec3{  9.0f,  9.0f,  9.0f };
const auto look_at_coord = kvs::Vec3{ 0, 0, 0};

const auto ViewDim1 = kvs::Vec3ui{ 1, 8, 1 }; // viewpoint dimension
const auto ViewDim2 = kvs::Vec3ui{ 1, 8, 2 }; // viewpoint dimension
const auto ViewDim3 = kvs::Vec3ui{ 1, 4, 2 }; // viewpoint dimension
const auto ViewDim4 = kvs::Vec3ui{ 1, 4, 3 }; // viewpoint dimension
const auto ViewDim5 = kvs::Vec3ui{ 1, 20, 1 }; // viewpoint dimension

const auto Viewpoint1 = InSituVis::PolyhedralViewpoint{ ViewDim1, m_min_coord, m_max_coord, look_at_coord, ViewDir };
const auto Viewpoint2 = InSituVis::PolyhedralViewpoint{ ViewDim2, m_min_coord, m_max_coord, look_at_coord, ViewDir };
const auto Viewpoint3 = InSituVis::PolyhedralViewpoint{ ViewDim3, m_min_coord, m_max_coord, look_at_coord, ViewDir };
const auto Viewpoint4 = InSituVis::PolyhedralViewpoint{ ViewDim4, m_min_coord, m_max_coord, look_at_coord, ViewDir };
const auto Viewpoint5 = InSituVis::PolyhedralViewpoint{ ViewDim5, m_min_coord, m_max_coord, look_at_coord, ViewDir };

} // end of namespace Params

/*****************************************************************************/

namespace local
{

class InSituVis : public ::Adaptor
{
    using BaseClass = ::Adaptor;
    using Object = BaseClass::Object;
    using Volume = kvs::UnstructuredVolumeObject;
    using Screen = BaseClass::Screen;

public:
    static Pipeline WholeMinMaxValues();
    static Pipeline OrthoSlice();
    static Pipeline Isosurface(const kvs::mpi::Communicator& world);
    static Pipeline ExternalFace( const kvs::mpi::Communicator& world );
    static Pipeline StochasticRendering( const size_t repeats );

private:
    kvs::PolygonObject m_boundary_mesh; ///< boundary mesh
    kvs::mpi::StampTimer m_sim_timer{ BaseClass::world() }; ///< timer for sim. process
    kvs::mpi::StampTimer m_cnv_timer{ BaseClass::world() }; ///< timer for cnv. process
    kvs::mpi::StampTimer m_vis_timer{ BaseClass::world() }; ///< timer for vis. process
    kvs::Real64 m_whole_min_value = 0.0; ///< min. value of whole time-varying volume data
    kvs::Real64 m_whole_max_value = 0.0; ///< max. value of whole time-varying volume data

public:
    InSituVis( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root )
    {
        // 1. Output Path Setting (from Sample Code)
        //--------------------------------------------------------------------
        {
            std::string base_dir =  "/data2/tomoya/OralAirFlow/test3.0";
            std::string sub_dir  = "Process";

            this->outputDirectory().setBaseDirectoryName( base_dir );
            this->outputDirectory().setSubDirectoryName( sub_dir );
        }

        // Common parameters.
        this->setImageSize( Params::ImageSize.x(), Params::ImageSize.y() );
        this->setOutputImageEnabled( Params::Output::Image );
        this->setOutputSubImageEnabled(
            Params::Output::SubImage,
            Params::Output::SubImageDepth,
            Params::Output::SubImageAlpha );

        // Import boundary mesh.
        this->importBoundaryMesh( "./constant/triSurface/realistic-cfd3.stl" );

        // Time intervals.
        this->setAnalysisInterval( Params::AnalysisInterval );

        // Set visualization pipeline.
#if defined( IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING )
        this->setRepetitionLevel( Params::Repeats );
        this->setPipeline( local::InSituVis::StochasticRendering( Params::Repeats ) );
#elif defined( IN_SITU_VIS__PIPELINE__ORTHO_SLICE )
        this->setPipeline( local::InSituVis::OrthoSlice() );
#elif defined( IN_SITU_VIS__PIPELINE__ISOSURFACE )
        this->setPipeline( local::InSituVis::Isosurface( BaseClass::world() ) );
#elif defined( IN_SITU_VIS__PIPELINE__EXTERNAL_FACE )
        this->setPipeline( local::InSituVis::ExternalFace( BaseClass::world() ) );
#endif

        // 2. Multiple Viewpoints Registration (from Sample Code)
        //--------------------------------------------------------------------
        // 以前の #if defined マクロによる単一視点登録を廃止し、複数を登録します。
        // 第2引数の文字列がサブディレクトリ名として使用されます。
        this->addViewpoint( Params::Viewpoint1, "181" );
        this->addViewpoint( Params::Viewpoint2, "182" );
        this->addViewpoint( Params::Viewpoint3, "142" );
        this->addViewpoint( Params::Viewpoint4, "143" );
        this->addViewpoint( Params::Viewpoint5, "1201" );
    }

    kvs::mpi::StampTimer& simTimer() { return m_sim_timer; }
    kvs::mpi::StampTimer& cnvTimer() { return m_cnv_timer; }
    kvs::mpi::StampTimer& visTimer() { return m_vis_timer; }

    void exec( const BaseClass::SimTime sim_time )
    {
        if ( !BaseClass::screen().scene()->hasObject( "BoundaryMesh") )
        {
            const bool visible = BaseClass::world().rank() == BaseClass::world().root ();
            auto* object = new kvs::PolygonObject();
            object->shallowCopy( m_boundary_mesh );
            object->setName( "BoundaryMesh" );
            object->setVisible( visible );

            // Register the bounding box at the root rank.
#if defined( IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING )
            // Bounding box
            kvs::Bounds bounds( kvs::RGBColor::Black(), 1.0f );
            auto* o = bounds.outputLineObject( object );
            o->setVisible( object->isVisible() );
            auto* r = new kvs::StochasticLineRenderer();
            BaseClass::screen().registerObject( o, r );

            // Boundary mesh
            object->setOpacity( Params::BoundaryMeshOpacity );
            auto* renderer = new kvs::StochasticPolygonRenderer();
            renderer->setTwoSideLightingEnabled( true );
            renderer->setEdgeFactor( 0.6f );
            BaseClass::screen().registerObject( object, renderer );
#else
            // Bounding box
            BaseClass::screen().registerObject( object, new kvs::Bounds() );
            object->setVisible( Params::VisibleBoundingBox );
#endif
        }

#if defined( IN_SITU_VIS__UPDATE_MIN_MAX_VALUES )
        // Update min/max values of the volume data in each time step.
        auto min_value = Volume::DownCast( *BaseClass::objects().begin() )->minValue();
        auto max_value = Volume::DownCast( *BaseClass::objects().begin() )->maxValue();
        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->updateMinMaxValues();

            min_value = kvs::Math::Min( min_value, volume->minValue() );
            max_value = kvs::Math::Max( max_value, volume->maxValue() );
        }

        BaseClass::world().allReduce( min_value, min_value, MPI_MIN );
        BaseClass::world().allReduce( max_value, max_value, MPI_MAX );

        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->setMinMaxValues( min_value, max_value );
        }
#endif

#if defined( IN_SITU_VIS__PIPELINE__WHOLE_MIN_MAX_VALUES )
        auto min_value = kvs::Value<kvs::Real64>::Max();
        auto max_value = kvs::Value<kvs::Real64>::Min();
        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->updateMinMaxValues();

            min_value = kvs::Math::Min( min_value, volume->minValue() );
            max_value = kvs::Math::Max( max_value, volume->maxValue() );
        }

        BaseClass::world().allReduce( min_value, min_value, MPI_MIN );
        BaseClass::world().allReduce( max_value, max_value, MPI_MAX );

        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->setMinMaxValues( min_value, max_value );
        }

        log() << "Min: " << min_value << std::endl;
        log() << "Max: " << max_value << std::endl;

        if ( sim_time.index == 1 )
        {
            m_whole_min_value = min_value;
            m_whole_max_value = max_value;
        }
        else
        {
            m_whole_min_value = kvs::Math::Min( m_whole_min_value, min_value );
            m_whole_max_value = kvs::Math::Max( m_whole_max_value, max_value );
        }

        log() << "Whole Min: " << m_whole_min_value << std::endl;
        log() << "Whole Max: " << m_whole_max_value << std::endl;
#endif

        BaseClass::exec( sim_time );
    }

    void importBoundaryMesh( const std::string& filename )
    {
        m_boundary_mesh = kvs::PolygonImporter( filename );

        // Scaling coordinate values of the boundary object adjusing to
        // the coordinate scale of the volume dataset.
        const auto scale = 1.0f / 1000.0f;
        {
            auto coords = m_boundary_mesh.coords();
            for ( auto& p : coords ) { p *= scale; }
            m_boundary_mesh.setCoords( coords );
        }

        const auto min_coord = m_boundary_mesh.minObjectCoord() * scale;
        const auto max_coord = m_boundary_mesh.maxObjectCoord() * scale;
        m_boundary_mesh.setMinMaxObjectCoords( min_coord, max_coord );
        m_boundary_mesh.setMinMaxExternalCoords( min_coord, max_coord );

        // Removing x-max, y-min/max, z-min/max planes and setting normals.
        {
            auto v = m_boundary_mesh.coords();
            std::vector<kvs::Real32> coords;
            std::vector<kvs::Real32> normals;
            for ( size_t i = 0; i < v.size() / 9; i++ )
            {
                const auto v0 = kvs::Vec3( v[9*i+0], v[9*i+1], v[9*i+2] );
                const auto v1 = kvs::Vec3( v[9*i+3], v[9*i+4], v[9*i+5] );
                const auto v2 = kvs::Vec3( v[9*i+6], v[9*i+7], v[9*i+8] );
                const auto n0 = ( v1 - v0 ).cross( v2 - v0 ).normalized();

                // x-max
                if ( kvs::Math::Equal( max_coord.x(), v0.x() ) &&
                     kvs::Math::Equal( max_coord.x(), v1.x() ) &&
                     kvs::Math::Equal( max_coord.x(), v2.x() ) ) continue;
                // y-min
                if ( kvs::Math::Equal( min_coord.y(), v0.y() ) &&
                     kvs::Math::Equal( min_coord.y(), v1.y() ) &&
                     kvs::Math::Equal( min_coord.y(), v2.y() ) ) continue;

                // y-max
                if ( kvs::Math::Equal( max_coord.y(), v0.y() ) &&
                     kvs::Math::Equal( max_coord.y(), v1.y() ) &&
                     kvs::Math::Equal( max_coord.y(), v2.y() ) ) continue;

                // z-min
                if ( kvs::Math::Equal( min_coord.z(), v0.z() ) &&
                     kvs::Math::Equal( min_coord.z(), v1.z() ) &&
                     kvs::Math::Equal( min_coord.z(), v2.z() ) ) continue;

                // z-max
                if ( kvs::Math::Equal( max_coord.z(), v0.z() ) &&
                     kvs::Math::Equal( max_coord.z(), v1.z() ) &&
                     kvs::Math::Equal( max_coord.z(), v2.z() ) ) continue;

                coords.push_back( v0.x() );
                coords.push_back( v0.y() );
                coords.push_back( v0.z() );

                coords.push_back( v1.x() );
                coords.push_back( v1.y() );
                coords.push_back( v1.z() );

                coords.push_back( v2.x() );
                coords.push_back( v2.y() );
                coords.push_back( v2.z() );

                normals.push_back( n0.x() );
                normals.push_back( n0.y() );
                normals.push_back( n0.z() );
            }
            m_boundary_mesh.setCoords( kvs::ValueArray<kvs::Real32>( coords ) );
            m_boundary_mesh.setNormals( kvs::ValueArray<kvs::Real32>( normals ) );
        }
    }

    bool dump()
    {
        if ( !BaseClass::dump() ) return false;

#if defined( IN_SITU_VIS__PIPELINE__WHOLE_MIN_MAX_VALUES )
        log() << "Whole Min: " << m_whole_min_value << std::endl;
        log() << "Whole Max: " << m_whole_max_value << std::endl;
#endif

        // For each node
        m_sim_timer.setTitle( "Sim time" );
        m_cnv_timer.setTitle( "Cnv time" );
        m_vis_timer.setTitle( "Vis time" );

        const std::string rank = kvs::String::From( this->world().rank(), 4, '0' );
        const std::string subdir = BaseClass::outputDirectory().name() + "/";
        kvs::StampTimerList timer_list;
        timer_list.push( m_sim_timer );
        timer_list.push( m_cnv_timer );
        timer_list.push( m_vis_timer );
        if ( !timer_list.write( subdir + "proc_time_" + rank + ".csv" ) ) return false;

        // For root node
        auto sim_time_min = m_sim_timer; sim_time_min.reduceMin();
        auto sim_time_max = m_sim_timer; sim_time_max.reduceMax();
        auto sim_time_ave = m_sim_timer; sim_time_ave.reduceAve();
        auto cnv_time_min = m_cnv_timer; cnv_time_min.reduceMin();
        auto cnv_time_max = m_cnv_timer; cnv_time_max.reduceMax();
        auto cnv_time_ave = m_cnv_timer; cnv_time_ave.reduceAve();
        auto vis_time_min = m_vis_timer; vis_time_min.reduceMin();
        auto vis_time_max = m_vis_timer; vis_time_max.reduceMax();
        auto vis_time_ave = m_vis_timer; vis_time_ave.reduceAve();

        if ( !this->world().isRoot() ) return true;

        sim_time_min.setTitle( "Sim time (min)" );
        sim_time_max.setTitle( "Sim time (max)" );
        sim_time_ave.setTitle( "Sim time (ave)" );
        cnv_time_min.setTitle( "Cnv time (min)" );
        cnv_time_max.setTitle( "Cnv time (max)" );
        cnv_time_ave.setTitle( "Cnv time (ave)" );
        vis_time_min.setTitle( "Vis time (min)" );
        vis_time_max.setTitle( "Vis time (max)" );
        vis_time_ave.setTitle( "Vis time (ave)" );

        timer_list.clear();
        timer_list.push( sim_time_min );
        timer_list.push( sim_time_max );
        timer_list.push( sim_time_ave );
        timer_list.push( cnv_time_min );
        timer_list.push( cnv_time_max );
        timer_list.push( cnv_time_ave );
        timer_list.push( vis_time_min );
        timer_list.push( vis_time_max );
        timer_list.push( vis_time_ave );

        const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
        return timer_list.write( basedir + "proc_time.csv" );
    }
};

inline InSituVis::Pipeline InSituVis::OrthoSlice()
{
    return [&] ( Screen& screen, const Object& object, const std::string& base_dir, int time_step )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();
        //auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        t.setRange( min_value, max_value );

        // Create new slice objects.
        auto p0 = ( volume.minObjectCoord().y() + volume.maxObjectCoord().y() ) * 0.5f;
        auto a0 = kvs::OrthoSlice::YAxis;
        auto* object0 = new kvs::OrthoSlice( &volume, p0, a0, t );
        object0->setName( volume.name() + "Object0");

        auto p1 = ( volume.minObjectCoord().z() + volume.maxObjectCoord().z() ) * 0.5f;
        auto a1 = kvs::OrthoSlice::ZAxis;
        auto* object1 = new kvs::OrthoSlice( &volume, p1, a1, t );
        object1->setName( volume.name() + "Object1");

        if ( screen.scene()->hasObject( volume.name() + "Object0") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object0", object0 );
            screen.scene()->replaceObject( volume.name() + "Object1", object1 );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer0 = new kvs::glsl::PolygonRenderer();
            auto* renderer1 = new kvs::glsl::PolygonRenderer();
            renderer0->setTwoSideLightingEnabled( true );
            renderer1->setTwoSideLightingEnabled( true );
            screen.registerObject( object0, renderer0 );
            screen.registerObject( object1, renderer1 );

            // Boundary mesh
            if ( Params::VisibleBoundaryMesh )
            {
                auto* renderer = new kvs::PolygonRenderer();
                renderer->setTwoSideLightingEnabled( true );
                screen.registerObject( mesh, renderer );
            }
        }
    };
}

inline InSituVis::Pipeline InSituVis::Isosurface(const kvs::mpi::Communicator& comm )
{
    return [comm] ( Screen& screen, const Object& object, const std::string& base_dir, int time_step )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();
        //auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        t.setRange( min_value, max_value );

        // Create new object
        auto n = kvs::Isosurface::PolygonNormal;
        auto d = true;
        auto i0 = kvs::Math::Mix( min_value, max_value, 0.1 );
        auto* object0 = new kvs::Isosurface( &volume, i0, n, d, t );
        object0->setName( volume.name() + "Object0");

        auto i1 = kvs::Math::Mix( min_value, max_value, 0.3 );
        auto* object1 = new kvs::Isosurface( &volume, i1, n, d, t );
        object1->setName( volume.name() + "Object1");

        auto i2 = kvs::Math::Mix( min_value, max_value, 0.7 );
        auto* object2 = new kvs::Isosurface( &volume, i2, n, d, t );
        object2->setName( volume.name() + "Object2");

        // Register object and renderer to screen
        kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object0") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object0", object0 );
            screen.scene()->replaceObject( volume.name() + "Object1", object1 );
            screen.scene()->replaceObject( volume.name() + "Object2", object2 );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer0 = new kvs::glsl::PolygonRenderer();
            auto* renderer1 = new kvs::glsl::PolygonRenderer();
            auto* renderer2 = new kvs::glsl::PolygonRenderer();
            renderer0->setTwoSideLightingEnabled( true );
            renderer1->setTwoSideLightingEnabled( true );
            renderer2->setTwoSideLightingEnabled( true );
            screen.registerObject( object0, renderer0 );
            screen.registerObject( object1, renderer1 );
            screen.registerObject( object2, renderer2 );

            // Boundary mesh
            if ( Params::VisibleBoundaryMesh )
            {
                auto* renderer = new kvs::PolygonRenderer();
                renderer->setTwoSideLightingEnabled( true );
                screen.registerObject( mesh, renderer );
            }
        }

        // ───────────────────────────────────────────────────────────
        // (2) ここから「視点に関わらず１回だけ行う処理」。
        //     Isosurface (Object0, Object1, Object2) をシーンから探して
        //     coords/colors/normals を取り出し、JSON ファイルへ書き出す
        // ───────────────────────────────────────────────────────────
        {
            std::vector<kvs::Vec3> all_coords;
            std::vector<kvs::RGBColor> all_colors;
            std::vector<kvs::Vec3> all_normals;

            int isosurface_count = 0;

            const auto& om = screen.scene()->objectManager();
            const int nobjects = om->numberOfObjects();
            for ( int i = 0; i < nobjects; ++i )
            {
                kvs::ObjectBase* obj = om->object( i );
                auto* poly = dynamic_cast<kvs::PolygonObject*>( obj );
                
                // 名前で判定 (Object0, Object1, Object2 などが含まれるか)
                // BoundaryMeshは除外
                if ( poly && ( poly->name().find( "Object" ) != std::string::npos ) && ( poly->name().find("BoundaryMesh") == std::string::npos ) )
                {
                    const auto& coords  = poly->coords();
                    const auto& colors  = poly->colors();
                    const auto& normals = poly->normals();

                    for ( size_t vi = 0; vi + 2 < coords.size(); vi += 3 )
                    {
                        kvs::Vec3 p_obj( coords[vi], coords[vi+1], coords[vi+2] );
                        kvs::Vec3 p_wld = kvs::ObjectCoordinate( p_obj, poly ).toWorldCoordinate().position();
                        all_coords.push_back( p_wld );
                    }
                    for ( size_t vi = 0; vi + 2 < colors.size(); vi += 3 )
                    {
                        all_colors.emplace_back( colors[vi], colors[vi+1], colors[vi+2] );
                    }
                    for ( size_t vi = 0; vi + 2 < normals.size(); vi += 3 )
                    {
                        kvs::Vec3 n_obj(normals[vi], normals[vi + 1], normals[vi + 2]);
                        kvs::Vec3 n_wld = kvs::ObjectCoordinate(n_obj, poly).toWorldCoordinate().position();
                        all_normals.push_back(n_wld);
                    }

                    ++isosurface_count;
                }
            }

            const int rank = comm.rank();
            std::cout << "[Rank " << rank << "] Found " << isosurface_count
                    << " Isosurface object(s), merged " << all_coords.size()
                    << " vertices.\n";

            if ( isosurface_count > 0 )
            {
                const std::string params_dir = base_dir + "/params";
                const std::string step_str = kvs::String::From( time_step, 6, '0' );
                const std::string rank_str = kvs::String::From( rank, 3, '0' );
                const std::string json_path = params_dir + "/isosurf_" + step_str + "_rank" + rank_str + ".json";
                
                // ディレクトリ作成
                {
                    struct stat st;
                    if ( ::stat( params_dir.c_str(), &st ) != 0 )
                    {
                        if ( ::mkdir( params_dir.c_str(), 0755 ) != 0 )
                        {
                            std::cerr << "Warning: Failed to create directory " << params_dir << "\n";
                        }
                    }
                }

                std::ofstream ofs( json_path );
                if ( !ofs.is_open() )
                {
                    std::cerr << "Error: Cannot open " << json_path << " for writing.\n";
                }
                else
                {
                    ofs << "{\n";
                    ofs << "  \"time_step\": " << time_step << ",\n";
                    ofs << "  \"rank\": " << rank << ",\n";

                    ofs << "  \"coords\": [\n";
                    for ( size_t i = 0; i < all_coords.size(); ++i )
                    {
                        const auto& v = all_coords[i];
                        ofs << "    [" << v.x() << ", " << v.y() << ", " << v.z() << "]";
                        if ( i + 1 < all_coords.size() ) ofs << ",";
                        ofs << "\n";
                    }
                    ofs << "  ],\n";

                    ofs << "  \"colors\": [\n";
                    for ( size_t i = 0; i < all_colors.size(); ++i )
                    {
                        const auto& c = all_colors[i];
                        ofs << "    [" << static_cast<int>(c.r()) << ", "
                                    << static_cast<int>(c.g()) << ", "
                                    << static_cast<int>(c.b()) << "]";
                        if ( i + 1 < all_colors.size() ) ofs << ",";
                        ofs << "\n";
                    }
                    ofs << "  ],\n";

                    ofs << "  \"normals\": [\n";
                    for ( size_t i = 0; i < all_normals.size(); ++i )
                    {
                        const auto& n = all_normals[i];
                        ofs << "    [" << n.x() << ", " << n.y() << ", " << n.z() << "]";
                        if ( i + 1 < all_normals.size() ) ofs << ",";
                        ofs << "\n";
                    }
                    ofs << "  ]\n";

                    ofs << "}\n";
                    ofs.close();

                    std::cout << "Wrote merged isosurface data to " << json_path << "\n";
                }
            }
        }
        // ───────────────────────────────────────────────────────────
        // (2) の処理 終わり
        // ───────────────────────────────────────────────────────────
    };
}

inline InSituVis::Pipeline InSituVis::ExternalFace( const kvs::mpi::Communicator& world )
{
    return [world] ( Screen& screen, const Object& object, const std::string& base_dir, int time_step )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        const auto cmap = kvs::ColorMap::BrewerSpectral( world.size() );
        auto indices = kvs::ValueArray<int>::Linear( world.size() );
        std::shuffle( indices.begin(), indices.end(), std::mt19937( 0 ) );
        const auto index = indices[ world.rank() ];
        const auto color = cmap[ index ];

        auto* faces = new kvs::ExternalFaces( &volume );
        faces->setName( volume.name() + "Object");
        faces->setColor( color );

        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", faces );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer = new kvs::glsl::PolygonRenderer();
            renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( faces, renderer );
        }
    };
}

inline InSituVis::Pipeline InSituVis::StochasticRendering( const size_t repeats )
{
    return [repeats] ( Screen& screen, const Object& object, const std::string& base_dir, int time_step )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();

        //auto c = kvs::ColorMap::CoolWarm( 256 );
        auto c = kvs::ColorMap::BrewerSpectral( 256 );
        auto o = kvs::OpacityMap( 256 );
        o.addPoint(   0, 0.0 );
        o.addPoint(  30, 0.0 );
        o.addPoint(  50, 0.2 );
        o.addPoint( 100, 0.3 );
        o.addPoint( 240, 0.2 );
        o.addPoint( 245, 0.1 );
        o.addPoint( 255, 0.0 );
        o.create();
        auto t = kvs::TransferFunction( c, o );
        //auto t = kvs::TransferFunction( c );
        t.setRange( min_value, max_value );

        // Particle generation.
        using Sampler = kvs::CellByCellMetropolisSampling;
        const auto* camera = screen.scene()->camera();
        const auto step = 0.5f / 1000.0f;
        auto* point = new Sampler( camera, &volume, repeats, step, t );
        point->setName( volume.name() + "Object");

        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            point->setMinMaxObjectCoords( min_coord, max_coord );
            point->setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Register object and renderer to screen
        //kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", point );
        }
        else
        {
            // Register the objects with renderer.
            auto* point_renderer = new kvs::glsl::ParticleBasedRenderer();
            point_renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( point, point_renderer );
        }
    };
}

} // end of namspace local