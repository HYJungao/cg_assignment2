#include "Radiosity.hpp"
#include "AreaLight.hpp"
#include "RayTracer.hpp"



namespace FW {


// --------------------------------------------------------------------------

bool Radiosity::m_QMC = false;
bool Radiosity::m_Stratified = false;

Radiosity::~Radiosity()
{
    if ( isRunning() )
    {
        m_context.m_bForceExit = true;
        while( m_launcher.getNumTasks() > m_launcher.getNumFinished() )
            Sleep( 1 );
        m_launcher.popAll();
    }
}


// --------------------------------------------------------------------------
void Radiosity::vertexTaskFunc( MulticoreLauncher::Task& task )
{
    RadiosityContext& ctx = *(RadiosityContext*)task.data;

    if( ctx.m_bForceExit )
        return;

    // which vertex are we to compute?
    int v = task.idx;

    // fetch vertex and its normal
    Vec3f n = ctx.m_scene->vertex(v).n.normalized();
    Vec3f o = ctx.m_scene->vertex(v).p + 0.01f*n;

    // YOUR CODE HERE (R3):
    // This starter code merely puts the color-coded normal into the result.
	// Remove the dummy solution to make your own implementation work.
    //
    // In the first bounce, your task is to compute the direct irradiance
    // falling on this vertex from the area light source.
    // In the subsequent passes, you should compute the irradiance by a
    // hemispherical gathering integral. The commented code below gives you
    // an idea of the loop structure. Note that you also have to account
    // for how diffuse textures modulate the irradiance.


	// This is the dummy implementation you should remove.

    Random rnd;
    // direct lighting pass? => integrate direct illumination by shooting shadow rays to light source
    if ( ctx.m_currentBounce == 0 )
    {
        Vec3f E(0);

        int row = FW::sqrt((float)ctx.m_numDirectRays);
        int col = ctx.m_numDirectRays / row;
        float dx = 2.0f / row;
        float dy = 2.0f / col;
        int i = 0;
        int j = 0;

        for ( int r = 0; r < ctx.m_numDirectRays; ++r )
        {
            // draw sample on light source
            float pdf;
            Vec3f Pl;
            if (m_QMC) {
                ctx.m_light->sampleHalton(pdf, Pl, 2, 3, r);
            }
            else if (m_Stratified) {
                if (r != 0 && r % row == 0) {
                    i = 0;
                    j++;
                }
                ctx.m_light->StratifiedSample(pdf, Pl, rnd, dx, dy, i, j);
                i++;
            }
            else {
                ctx.m_light->sample(pdf, Pl, 0, rnd);
            }

            // construct vector from current vertex (o) to light sample
            Vec3f v2l = Pl - o;

            // trace shadow ray to see if it's blocked
            if (!ctx.m_rt->raycast(o, v2l))
            {
                // if not, add the appropriate emission, 1/r^2 and clamped cosine terms, accounting for the PDF as well.
                // accumulate into E
                float cosTheta = FW::clamp(FW::dot(-v2l.normalized(), ctx.m_light->getNormal()), 0.0f, 1.0f);
                float cosThetaY = FW::clamp(FW::dot(v2l.normalized(), n), 0.0f, 1.0f);
                E += ctx.m_light->getEmission() * cosTheta * cosThetaY / (v2l.lenSqr() * pdf);
            }
        }
        // Note we are NOT multiplying by PI here;
        // it's implicit in the hemisphere-to-light source area change of variables.
        // The result we are computing is _irradiance_, not radiosity.
        ctx.m_vecCurr[ v ] = E * (1.0f/ctx.m_numDirectRays);
        ctx.m_vecResult[ v ] = ctx.m_vecCurr[ v ];
    }
    else
    {
        // OK, time for indirect!
        // Implement hemispherical gathering integral for bounces > 1.

        // Get local coordinate system the rays are shot from.
        Mat3f B = formBasis( n );

        Vec3f E(0.0f);
        for ( int r = 0; r < ctx.m_numHemisphereRays; ++r )
        {
            // Draw a cosine weighted direction and find out where it hits (if anywhere)
            // You need to transform it from the local frame to the vertex' hemisphere using B.
            float a;
            float b;

            if (m_QMC) {
                a = halton(r, 5);
                b = halton(r, 7);
            }
            else
            {
                a = rnd.getF32(0, 1);
                b = rnd.getF32(0, 1);
            }

            float radial = FW::sqrt(a);
            float theta = 2.0 * FW_PI * b;

            float x = radial * FW::cos(theta);
            float y = radial * FW::sin(theta);

            Vec3f d(x, y, FW::sqrt(1 - a));

            // Make the direction long but not too long to avoid numerical instability in the ray tracer.
            // For our scenes, 100 is a good length. (I know, this special casing sucks.)
            d = B * d * 100.f;

            // Shoot ray, see where we hit
            const RaycastResult result = ctx.m_rt->raycast( o, d );
            if ( result.tri != nullptr )
            {
                // interpolate lighting from previous pass
				const Vec3i& indices = result.tri->m_data.vertex_indices;

                // check for backfaces => don't accumulate if we hit a surface from below!
                if (FW::dot(d, result.tri->normal()) > 0) {
                    continue;
                }

                // fetch barycentric coordinates

                // Ei = interpolated irradiance determined by ctx.m_vecPrevBounce from vertices using the barycentric coordinates
                Vec3f Ei = (1.f - result.u - result.v) * ctx.m_vecPrevBounce[indices[0]] + result.u * ctx.m_vecPrevBounce[indices[1]] + result.v * ctx.m_vecPrevBounce[indices[2]];

                // Divide incident irradiance by PI so that we can turn it into outgoing
                // radiosity by multiplying by the reflectance factor below.
                Ei *= (1.0f / FW_PI);

                // check for texture
                const auto mat = result.tri->m_material;
                if ( mat->textures[MeshBase::TextureType_Diffuse].exists() )
                {
					
					// read diffuse texture like in assignment1

                    const Texture& tex = mat->textures[MeshBase::TextureType_Diffuse];
                    const Image& teximg = *tex.getImage();

                    Vec2f uv0 = result.tri->m_vertices[0].t;
                    Vec2f uv1 = result.tri->m_vertices[1].t;
                    Vec2f uv2 = result.tri->m_vertices[2].t;

                    Vec2f uv = (1 - result.u - result.v) * uv0 + result.u * uv1 + result.v * uv2;

                    Vec2i texelCoords = getTexelCoords(uv, teximg.getSize());
                    Ei *= teximg.getVec4f(texelCoords).getXYZ();
                }
                else
                {
                    // no texture, use constant albedo from material structure.
                    // (this is just one line)
                    Ei *= result.tri->m_material->diffuse.getXYZ();
                }

                E += Ei;	// accumulate
            }
        }
        // Store result for this bounce
        // Note that since we are storing irradiance, we multiply by PI(
        // (Remember the slides about cosine weighted importance sampling!)
        ctx.m_vecCurr[ v ] = E * (FW_PI / ctx.m_numHemisphereRays);
        // Also add to the global accumulator.
        ctx.m_vecResult[ v ] = ctx.m_vecResult[ v ] + ctx.m_vecCurr[ v ];

        // uncomment this to visualize only the current bounce
        // ctx.m_vecResult[ v ] = ctx.m_vecCurr[ v ];	
    }
}
// --------------------------------------------------------------------------

void Radiosity::startRadiosityProcess( MeshWithColors* scene, AreaLight* light, RayTracer* rt, int numBounces, int numDirectRays, int numHemisphereRays )
{
    m_showIntermediateResult = false;
    m_isFinished = false;
    m_context.m_intermediateResult.clear();

    // put stuff the asyncronous processor needs 
    m_context.m_scene				= scene;
    m_context.m_rt					= rt;
    m_context.m_light				= light;
    m_context.m_currentBounce		= 0;
    m_context.m_numBounces			= numBounces;
    m_context.m_numDirectRays		= numDirectRays;
    m_context.m_numHemisphereRays	= numHemisphereRays;

    // resize all the buffers according to how many vertices we have in the scene
	m_context.m_vecResult.resize(scene->numVertices());
    m_context.m_vecCurr.resize( scene->numVertices() );
    m_context.m_vecPrevBounce.resize( scene->numVertices() );
    m_context.m_vecResult.assign( scene->numVertices(), Vec3f(0,0,0) );

	m_context.m_vecSphericalC.resize(scene->numVertices());
	m_context.m_vecSphericalX.resize(scene->numVertices());
	m_context.m_vecSphericalY.resize(scene->numVertices());
	m_context.m_vecSphericalZ.resize(scene->numVertices());

	m_context.m_vecSphericalC.assign(scene->numVertices(), Vec3f(0, 0, 0));
	m_context.m_vecSphericalX.assign(scene->numVertices(), Vec3f(0, 0, 0));
	m_context.m_vecSphericalY.assign(scene->numVertices(), Vec3f(0, 0, 0));
	m_context.m_vecSphericalZ.assign(scene->numVertices(), Vec3f(0, 0, 0));

    // fire away!
    m_launcher.setNumThreads(m_launcher.getNumCores());	// the solution exe is multithreaded
    // m_launcher.setNumThreads(1);							// but you have to make sure your code is thread safe before enabling this!
    m_launcher.popAll();
    m_launcher.push( vertexTaskFunc, &m_context, 0, scene->numVertices() );
}
// --------------------------------------------------------------------------

bool Radiosity::updateMeshColors(std::vector<Vec4f>& spherical1, std::vector<Vec4f>& spherical2, std::vector<float>& spherical3, bool spherical)
{
	if (!m_context.m_scene || m_context.m_vecResult.size()==0) return false;
    // Print progress.
    printf( "%.2f%% done     \r", 100.0f*m_launcher.getNumFinished()/m_context.m_scene->numVertices() );

    // Copy irradiance over to the display mesh.
    // Because we want outgoing radiosity in the end, we divide by PI here
    // and let the shader multiply the final diffuse reflectance in. See App::setupShaders() for details.
	for (int i = 0; i < m_context.m_scene->numVertices(); ++i) {

		// Packing data for the spherical harmonic extra.
		// In order to manage with fewer vertex attributes in the shader, the third component is stored as the w components of other actually three-dimensional vectors.
		if (spherical) {
			m_context.m_scene->mutableVertex(i).c = m_context.m_vecSphericalC[i] * (1.0f / FW_PI);
			spherical3[i] = m_context.m_vecSphericalZ[i].x * (1.0f / FW_PI);
			spherical1[i] = Vec4f(m_context.m_vecSphericalX[i], m_context.m_vecSphericalZ[i].y) * (1.0f / FW_PI);
			spherical2[i] = Vec4f(m_context.m_vecSphericalY[i], m_context.m_vecSphericalZ[i].z) * (1.0f / FW_PI);
		}
		else {
			m_context.m_scene->mutableVertex(i).c = m_context.m_vecResult[i] * (1.0f / FW_PI);
		}
	}
	return true;
}
// --------------------------------------------------------------------------

void Radiosity::checkFinish()
{
    // have all the vertices from current bounce finished computing?
    if ( m_launcher.getNumTasks() == m_launcher.getNumFinished() )
    {
        // yes, remove from task list
        m_launcher.popAll();

        m_context.m_intermediateResult.push_back(m_context.m_vecCurr);
        // more bounces desired?
        if ( m_context.m_currentBounce < m_context.m_numBounces )
        {
            // move current bounce to prev
            m_context.m_vecPrevBounce = m_context.m_vecCurr;

            ++m_context.m_currentBounce;
            // start new tasks for all vertices
            m_launcher.push( vertexTaskFunc, &m_context, 0, m_context.m_scene->numVertices() );
            printf( "\nStarting bounce %d\n", m_context.m_currentBounce );
        }
        else {
            m_context.m_BackupResult = m_context.m_vecResult;
            m_isFinished = true;
            printf("\n DONE!\n");
        }
    }
}
// --------------------------------------------------------------------------

void Radiosity::showIntermediateResult(int round)
{
    if (m_isFinished && round < m_context.m_intermediateResult.size()) {
        if (m_showIntermediateResult == false) {
            m_showIntermediateResult = true;
            m_context.m_vecResult = m_context.m_intermediateResult.at(round);
        }
        else {
            m_showIntermediateResult = false;
            m_context.m_vecResult = m_context.m_BackupResult;
        }
    }
    else {
        printf("\n Computing NOT DONE!\n");
    }
}

float Radiosity::halton(int index, int base) {
    float result = 0;
    float f = 1.0 / base;
    int i = index;
    while (i > 0) {
        result = result + f * (i % base);
        i = i / base;
        f = f / base;
    }
    return result;
}

} // namespace FW
