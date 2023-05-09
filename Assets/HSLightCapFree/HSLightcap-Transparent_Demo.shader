// Made with Amplify Shader Editor
// Available at the Unity Asset Store - http://u3d.as/y3X 
Shader "HoneyShibari/HSLightcap_Standard_Demo"
{
	Properties
	{
		[Toggle(_USETEXTURE_ON)] _Usetexture("Use texture", Float) = 1
		[Toggle(_USETINT_ON)] _Usetint("Use tint", Float) = 0
		[Toggle(_USENORMAL_ON)] _Usenormal("Use normal", Float) = 0
		[Toggle(_USEDETAIL_ON)] _Usedetail("Use detail", Float) = 0
		_TintPower("Tint Power", Range( 0 , 1)) = 0.2
		_NormalPower("Normal Power", Range( 0 , 1)) = 0.5
		_LightcapPower("Lightcap Power", Range( 0 , 2)) = 1
		_DetailPower("Detail Power", Range( 0 , 1)) = 0.2
		_LightcapAngle("Lightcap Angle", Range( 0 , 360)) = 0.5
		_TintColor("Tint Color", Color) = (0.5019608,0.5019608,0.5019608,0)
		_BaseTexture("_BaseTexture", 2D) = "white" {}
		_LightcapTexture("Lightcap Texture", 2D) = "white" {}
		_NormalTexture("Normal Texture", 2D) = "bump" {}
		[Toggle(_SHADOWMAP_ON)] _ShadowMap("ShadowMap", Float) = 0
		[Toggle(_DEMO_ON)] _Demo("Demo", Float) = 1
		[Toggle(_SHADOWS_ON)] _Shadows("Shadows", Float) = 0
		[Toggle(_REALTIMESHADOWS_ON)] _RealtimeShadows("Realtime Shadows", Float) = 0
		_ShadowMapColor("ShadowMap Color", Color) = (0,0,0,0)
		_DetailTexture("Detail Texture", 2D) = "white" {}
		[Toggle(_BACKGROUND_ON)] _Background("Background", Float) = 0
		_BrightTones("Bright Tones", Color) = (0.9843137,0,0.02745098,1)
		_WhiteTones("White Tones", Color) = (0.3607843,1,0.3176471,1)
		_DarkTones("Dark Tones", Color) = (0.0627451,0.4784314,0.4745098,1)
		_BlackTones("Black Tones", Color) = (0,0,0,1)
		_MidTones("Mid Tones", Color) = (0.4705882,0.4509804,0.4627451,1)
		[Toggle(_LIGHTCAPCOLORS_ON)] _LightcapColors("Lightcap Colors", Float) = 0
		_TonesBlend("Tones Blend", Range( 0 , 1)) = 0.5
		[HideInInspector] _texcoord( "", 2D ) = "white" {}
		[HideInInspector] __dirty( "", Int ) = 1
	}

	SubShader
	{
		Tags{ "RenderType" = "Opaque"  "Queue" = "Geometry+0" "IgnoreProjector" = "True" }
		Cull Back
		CGINCLUDE
		#include "UnityPBSLighting.cginc"
		#include "UnityShaderVariables.cginc"
		#include "Lighting.cginc"
		#pragma target 3.0
		#pragma shader_feature_local _DEMO_ON
		#pragma multi_compile_local __ _SHADOWS_ON
		#pragma multi_compile_local __ _REALTIMESHADOWS_ON
		#pragma multi_compile_local __ _SHADOWMAP_ON
		#pragma multi_compile_local __ _LIGHTCAPCOLORS_ON
		#pragma multi_compile_local __ _USETEXTURE_ON
		#pragma multi_compile_local __ _USENORMAL_ON
		#pragma multi_compile_local __ _BACKGROUND_ON
		#pragma multi_compile_local __ _USEDETAIL_ON
		#pragma multi_compile_local __ _USETINT_ON
		struct Input
		{
			half3 worldNormal;
			float2 uv_texcoord;
		};

		struct SurfaceOutputCustomLightingCustom
		{
			half3 Albedo;
			half3 Normal;
			half3 Emission;
			half Metallic;
			half Smoothness;
			half Occlusion;
			half Alpha;
			Input SurfInput;
			UnityGIInput GIData;
		};

		uniform sampler2D _LightcapTexture;
		uniform float _LightcapAngle;
		uniform sampler2D _BaseTexture;
		uniform half4 _BaseTexture_ST;
		uniform half _LightcapPower;
		uniform half4 _TintColor;
		uniform sampler2D _NormalTexture;
		uniform half4 _NormalTexture_ST;
		uniform half4 _MidTones;
		uniform half4 _DarkTones;
		uniform half4 _BlackTones;
		uniform half4 _BrightTones;
		uniform half4 _WhiteTones;
		uniform half4 _ShadowMapColor;
		uniform sampler2D _DetailTexture;
		uniform half4 _DetailTexture_ST;
		uniform half _TonesBlend;
		uniform float _NormalPower;
		uniform half _DetailPower;
		uniform half _TintPower;


		float4 CalculateContrast( float contrastValue, float4 colorTarget )
		{
			float t = 0.5 * ( 1.0 - contrastValue );
			return mul( float4x4( contrastValue,0,0,t, 0,contrastValue,0,t, 0,0,contrastValue,t, 0,0,0,1 ), colorTarget );
		}

		inline half4 LightingStandardCustomLighting( inout SurfaceOutputCustomLightingCustom s, half3 viewDir, UnityGI gi )
		{
			UnityGIInput data = s.GIData;
			Input i = s.SurfInput;
			half4 c = 0;
			half3 ase_worldNormal = i.worldNormal;
			float cos254 = cos( ( _LightcapAngle / 57.50799 ) );
			float sin254 = sin( ( _LightcapAngle / 57.50799 ) );
			half2 rotator254 = mul( ( ( mul( UNITY_MATRIX_V, half4( ase_worldNormal , 0.0 ) ).xyz * 0.5 ) + 0.5 ).xy - float2( 0.5,0.5 ) , float2x2( cos254 , -sin254 , sin254 , cos254 )) + float2( 0.5,0.5 );
			half4 tex2DNode226 = tex2D( _LightcapTexture, rotator254 );
			#ifdef _LIGHTCAPCOLORS_ON
				half4 staticSwitch416 = tex2DNode226;
			#else
				half4 staticSwitch416 = tex2DNode226;
			#endif
			half4 color530 = IsGammaSpace() ? half4(0.4235294,0.4235294,0.4235294,0) : half4(0.1499598,0.1499598,0.1499598,0);
			float2 uv_BaseTexture = i.uv_texcoord * _BaseTexture_ST.xy + _BaseTexture_ST.zw;
			#ifdef _USETEXTURE_ON
				half4 staticSwitch168 = tex2D( _BaseTexture, uv_BaseTexture );
			#else
				half4 staticSwitch168 = color530;
			#endif
			half4 blendOpSrc151 = staticSwitch416;
			half4 blendOpDest151 = staticSwitch168;
			half4 lerpBlendMode151 = lerp(blendOpDest151, (( blendOpSrc151 > 0.5 ) ? ( 1.0 - ( 1.0 - 2.0 * ( blendOpSrc151 - 0.5 ) ) * ( 1.0 - blendOpDest151 ) ) : ( 2.0 * blendOpSrc151 * blendOpDest151 ) ),_LightcapPower);
			half4 blendOpSrc373 = staticSwitch416;
			half4 blendOpDest373 = staticSwitch168;
			half4 lerpBlendMode373 = lerp(blendOpDest373,(( blendOpSrc373 > 0.5 ) ? ( blendOpDest373 / max( ( 1.0 - blendOpSrc373 ) * 2.0 ,0.00001) ) : ( 1.0 - ( ( ( 1.0 - blendOpDest373 ) * 0.5 ) / max( blendOpSrc373,0.00001) ) ) ),_LightcapPower);
			half4 color512 = IsGammaSpace() ? half4(0.5019608,0.5019608,0.5019608,1) : half4(0.2158605,0.2158605,0.2158605,1);
			half grayscale510 = Luminance(tex2DNode226.rgb);
			half4 temp_cast_5 = (grayscale510).xxxx;
			half4 temp_cast_7 = (saturate( ( 1.0 - ( ( distance( color512.rgb , CalculateContrast(2.0,temp_cast_5).rgb ) - 0.0 ) / max( 1.0 , 1E-05 ) ) ) )).xxxx;
			half4 lerpResult372 = lerp( lerpBlendMode151 , lerpBlendMode373 , CalculateContrast(0.5,temp_cast_7));
			#ifdef _SHADOWMAP_ON
				half4 staticSwitch303 = lerpResult372;
			#else
				half4 staticSwitch303 = lerpResult372;
			#endif
			#ifdef _REALTIMESHADOWS_ON
				half4 staticSwitch509 = staticSwitch303;
			#else
				half4 staticSwitch509 = staticSwitch303;
			#endif
			#ifdef _SHADOWS_ON
				half4 staticSwitch508 = staticSwitch509;
			#else
				half4 staticSwitch508 = staticSwitch509;
			#endif
			float2 uv_NormalTexture = i.uv_texcoord * _NormalTexture_ST.xy + _NormalTexture_ST.zw;
			half4 lerpResult461 = lerp( _MidTones , _DarkTones , float4( 0,0,0,0 ));
			half4 lerpResult464 = lerp( lerpResult461 , _BlackTones , float4( 0,0,0,0 ));
			half4 lerpResult469 = lerp( lerpResult464 , _BrightTones , float4( 0,0,0,0 ));
			half4 lerpResult470 = lerp( lerpResult469 , _WhiteTones , _ShadowMapColor);
			float2 uv_DetailTexture = i.uv_texcoord * _DetailTexture_ST.xy + _DetailTexture_ST.zw;
			half4 lerpResult521 = lerp( half4( UnpackNormal( tex2D( _NormalTexture, uv_NormalTexture ) ) , 0.0 ) , lerpResult470 , tex2D( _DetailTexture, uv_DetailTexture ));
			half4 blendOpSrc516 = _TintColor;
			half4 blendOpDest516 = lerpResult521;
			half lerpResult522 = lerp( _TonesBlend , _NormalPower , _DetailPower);
			half lerpResult529 = lerp( lerpResult522 , _TintPower , 0.0);
			half4 lerpBlendMode516 = lerp(blendOpDest516,( 1.0 - ( ( 1.0 - blendOpDest516) / max( blendOpSrc516, 0.00001) ) ),lerpResult529);
			#ifdef _USETINT_ON
				half staticSwitch167 = 0.0;
			#else
				half staticSwitch167 = 0.0;
			#endif
			#ifdef _USEDETAIL_ON
				half staticSwitch421 = staticSwitch167;
			#else
				half staticSwitch421 = staticSwitch167;
			#endif
			#ifdef _BACKGROUND_ON
				half staticSwitch316 = staticSwitch421;
			#else
				half staticSwitch316 = 0.0;
			#endif
			#ifdef _USENORMAL_ON
				half staticSwitch196 = staticSwitch316;
			#else
				half staticSwitch196 = staticSwitch316;
			#endif
			half4 temp_cast_9 = (staticSwitch196).xxxx;
			half4 blendOpSrc517 = ( saturate( lerpBlendMode516 ));
			half4 blendOpDest517 = temp_cast_9;
			half4 lerpResult526 = lerp( staticSwitch508 , ( saturate( ( 1.0 - ( ( 1.0 - blendOpDest517) / max( blendOpSrc517, 0.00001) ) ) )) , float4( 0,0,0,0 ));
			#ifdef _DEMO_ON
				half4 staticSwitch525 = staticSwitch508;
			#else
				half4 staticSwitch525 = lerpResult526;
			#endif
			c.rgb = staticSwitch525.rgb;
			c.a = 1;
			return c;
		}

		inline void LightingStandardCustomLighting_GI( inout SurfaceOutputCustomLightingCustom s, UnityGIInput data, inout UnityGI gi )
		{
			s.GIData = data;
		}

		void surf( Input i , inout SurfaceOutputCustomLightingCustom o )
		{
			o.SurfInput = i;
		}

		ENDCG
		CGPROGRAM
		#pragma surface surf StandardCustomLighting keepalpha fullforwardshadows noambient novertexlights nolightmap  nodynlightmap nodirlightmap nometa noforwardadd 

		ENDCG
		Pass
		{
			Name "ShadowCaster"
			Tags{ "LightMode" = "ShadowCaster" }
			ZWrite On
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma target 3.0
			#pragma multi_compile_shadowcaster
			#pragma multi_compile UNITY_PASS_SHADOWCASTER
			#pragma skip_variants FOG_LINEAR FOG_EXP FOG_EXP2
			#include "HLSLSupport.cginc"
			#if ( SHADER_API_D3D11 || SHADER_API_GLCORE || SHADER_API_GLES || SHADER_API_GLES3 || SHADER_API_METAL || SHADER_API_VULKAN )
				#define CAN_SKIP_VPOS
			#endif
			#include "UnityCG.cginc"
			#include "Lighting.cginc"
			#include "UnityPBSLighting.cginc"
			struct v2f
			{
				V2F_SHADOW_CASTER;
				float2 customPack1 : TEXCOORD1;
				float3 worldPos : TEXCOORD2;
				float3 worldNormal : TEXCOORD3;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};
			v2f vert( appdata_full v )
			{
				v2f o;
				UNITY_SETUP_INSTANCE_ID( v );
				UNITY_INITIALIZE_OUTPUT( v2f, o );
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO( o );
				UNITY_TRANSFER_INSTANCE_ID( v, o );
				Input customInputData;
				float3 worldPos = mul( unity_ObjectToWorld, v.vertex ).xyz;
				half3 worldNormal = UnityObjectToWorldNormal( v.normal );
				o.worldNormal = worldNormal;
				o.customPack1.xy = customInputData.uv_texcoord;
				o.customPack1.xy = v.texcoord;
				o.worldPos = worldPos;
				TRANSFER_SHADOW_CASTER_NORMALOFFSET( o )
				return o;
			}
			half4 frag( v2f IN
			#if !defined( CAN_SKIP_VPOS )
			, UNITY_VPOS_TYPE vpos : VPOS
			#endif
			) : SV_Target
			{
				UNITY_SETUP_INSTANCE_ID( IN );
				Input surfIN;
				UNITY_INITIALIZE_OUTPUT( Input, surfIN );
				surfIN.uv_texcoord = IN.customPack1.xy;
				float3 worldPos = IN.worldPos;
				half3 worldViewDir = normalize( UnityWorldSpaceViewDir( worldPos ) );
				surfIN.worldNormal = IN.worldNormal;
				SurfaceOutputCustomLightingCustom o;
				UNITY_INITIALIZE_OUTPUT( SurfaceOutputCustomLightingCustom, o )
				surf( surfIN, o );
				#if defined( CAN_SKIP_VPOS )
				float2 vpos = IN.pos;
				#endif
				SHADOW_CASTER_FRAGMENT( IN )
			}
			ENDCG
		}
	}
	Fallback "Diffuse"
	CustomEditor "HSLightcapDemo_Editor"
}
/*ASEBEGIN
Version=17800
0;488;1149;373;5975.83;4996.354;15.69257;True;True
Node;AmplifyShaderEditor.CommentaryNode;128;22.5532,-2941.013;Inherit;False;2291.328;1158.503;;13;226;254;228;227;50;344;48;47;416;436;510;168;530;Matcap;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;344;309.6286,-2519.812;Inherit;False;549.8301;190.5432;;3;326;321;49;;1,1,1,1;0;0
Node;AmplifyShaderEditor.ViewMatrixNode;47;178.2099,-2470.613;Inherit;False;0;1;FLOAT4x4;0
Node;AmplifyShaderEditor.WorldNormalVector;48;115.5729,-2372.306;Inherit;False;False;1;0;FLOAT3;0,0,0;False;4;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3
Node;AmplifyShaderEditor.SimpleMultiplyOpNode;49;338.6576,-2467.898;Inherit;False;2;2;0;FLOAT4x4;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;False;1;FLOAT3;0,0,0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.RangedFloatNode;50;156.475,-2227.407;Float;False;Constant;_Float0;Float 0;-1;0;Create;True;0;0;False;0;0.5;0;0;0;0;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleMultiplyOpNode;321;494.0968,-2466.119;Inherit;False;2;2;0;FLOAT3;0,0,0;False;1;FLOAT;0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.RangedFloatNode;227;885.191,-2342.639;Float;False;Property;_LightcapAngle;Lightcap Angle;8;0;Create;True;0;0;False;0;0.5;225;0;360;0;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleDivideOpNode;228;1185.896,-2306.974;Inherit;False;2;0;FLOAT;0;False;1;FLOAT;57.50799;False;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleAddOpNode;326;711.7237,-2471.155;Inherit;False;2;2;0;FLOAT3;0,0,0;False;1;FLOAT;0;False;1;FLOAT3;0
Node;AmplifyShaderEditor.RotatorNode;254;1336.29,-2331.805;Inherit;False;3;0;FLOAT2;0,0;False;1;FLOAT2;0.5,0.5;False;2;FLOAT;1;False;1;FLOAT2;0
Node;AmplifyShaderEditor.CommentaryNode;523;2109.063,-1573.819;Inherit;False;2047.524;1347.765;;26;517;196;516;521;112;529;316;152;470;412;421;155;522;468;469;437;156;451;295;463;464;167;462;461;456;455;Demo;1,1,1,1;0;0
Node;AmplifyShaderEditor.SamplerNode;226;1532.139,-2351.755;Inherit;True;Property;_LightcapTexture;Lightcap Texture;11;0;Create;True;0;0;False;0;-1;eff7e93b4e8764437b64c4865f0ed016;c2a3f635689c14e188c3f4605c0a510f;True;0;False;white;Auto;False;Object;-1;Auto;Texture2D;6;0;SAMPLER2D;0,0;False;1;FLOAT2;1,0;False;2;FLOAT;1;False;3;FLOAT2;0,0;False;4;FLOAT2;0,0;False;5;FLOAT;1;False;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.ColorNode;456;2564.474,-879.7033;Inherit;False;Property;_MidTones;Mid Tones;24;0;Create;True;0;0;False;0;0.4705882,0.4509804,0.4627451,1;0.4705882,0.4509804,0.4627451,1;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.CommentaryNode;436;111.3046,-2860.092;Inherit;False;368.5;280;Comment;1;106;Texture;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;194;2590.594,-2791.862;Inherit;False;1414.151;993.8682;;8;372;151;511;373;195;514;513;512;Matcap Tex Union;1,1,1,1;0;0
Node;AmplifyShaderEditor.TFHCGrayscale;510;1991.985,-2198.397;Inherit;True;0;1;0;FLOAT3;0,0,0;False;1;FLOAT;0
Node;AmplifyShaderEditor.ColorNode;455;2572.095,-856.2028;Inherit;False;Property;_DarkTones;Dark Tones;22;0;Create;True;0;0;False;0;0.0627451,0.4784314,0.4745098,1;0.02745098,0.2196078,0.2392157,1;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.SimpleContrastOpNode;513;2649.928,-2346.89;Inherit;True;2;1;COLOR;0,0,0,0;False;0;FLOAT;2;False;1;COLOR;0
Node;AmplifyShaderEditor.SamplerNode;106;146.4035,-2783.27;Inherit;True;Property;_BaseTexture;_BaseTexture;10;0;Create;True;0;0;False;0;-1;e1474879f3f2b473e9b748c8be439bca;9d5c57e3f17af5b44bd6441ead7a15a8;True;0;False;white;Auto;False;Object;-1;Auto;Texture2D;6;0;SAMPLER2D;;False;1;FLOAT2;0,0;False;2;FLOAT;0;False;3;FLOAT2;0,0;False;4;FLOAT2;0,0;False;5;FLOAT;1;False;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.ColorNode;530;570.463,-2849.358;Inherit;False;Constant;_Color1;Color 1;27;0;Create;True;0;0;False;0;0.4235294,0.4235294,0.4235294,0;0,0,0,0;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.ColorNode;462;2565.472,-957.8208;Inherit;False;Property;_BlackTones;Black Tones;23;0;Create;True;0;0;False;0;0,0,0,1;0,0,0,1;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.LerpOp;461;2883.763,-975.0693;Inherit;True;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.ColorNode;512;2680.522,-2114.241;Inherit;False;Constant;_Color0;Color 0;27;0;Create;True;0;0;False;0;0.5019608,0.5019608,0.5019608,1;0,0,0,0;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.StaticSwitch;416;1955.577,-2374.249;Inherit;False;Property;_LightcapColors;Lightcap Colors;25;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;COLOR;0,0,0,0;False;0;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;3;COLOR;0,0,0,0;False;4;COLOR;0,0,0,0;False;5;COLOR;0,0,0,0;False;6;COLOR;0,0,0,0;False;7;COLOR;0,0,0,0;False;8;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.RangedFloatNode;195;2672.68,-2548.789;Inherit;False;Property;_LightcapPower;Lightcap Power;6;0;Create;True;0;0;False;0;1;1;0;2;0;1;FLOAT;0
Node;AmplifyShaderEditor.StaticSwitch;168;918.1205,-2695.123;Inherit;False;Property;_Usetexture;Use texture;0;0;Create;True;0;0;False;0;1;1;1;True;;Toggle;2;Key0;Key1;Create;True;9;1;COLOR;0,0,0,0;False;0;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;3;COLOR;0,0,0,0;False;4;COLOR;0,0,0,0;False;5;COLOR;0,0,0,0;False;6;COLOR;0,0,0,0;False;7;COLOR;0,0,0,0;False;8;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.LerpOp;464;2885.654,-1060.823;Inherit;True;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.ColorNode;463;2559.952,-1029.102;Inherit;False;Property;_BrightTones;Bright Tones;20;0;Create;True;0;0;False;0;0.9843137,0,0.02745098,1;0.9843137,0.572549,0.2039216,1;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.FunctionNode;514;3030.506,-2193.355;Inherit;True;Color Mask;-1;;38;eec747d987850564c95bde0e5a6d1867;0;4;1;FLOAT3;0,0,0;False;3;FLOAT3;0,0,0;False;4;FLOAT;0;False;5;FLOAT;1;False;1;FLOAT;0
Node;AmplifyShaderEditor.ColorNode;295;2584.595,-1236.171;Inherit;False;Property;_ShadowMapColor;ShadowMap Color;17;0;Create;True;0;0;False;0;0,0,0,0;0,0,0,0;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.RangedFloatNode;451;2580.036,-690.2395;Inherit;False;Property;_TonesBlend;Tones Blend;26;0;Create;False;0;0;False;0;0.5;0.718;0;1;0;1;FLOAT;0
Node;AmplifyShaderEditor.LerpOp;469;2885.539,-1104.811;Inherit;True;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.BlendOpsNode;151;3049.819,-2710.743;Inherit;True;HardLight;False;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;FLOAT;1;False;1;COLOR;0
Node;AmplifyShaderEditor.ColorNode;468;2562.587,-1096.908;Inherit;False;Property;_WhiteTones;White Tones;21;0;Create;True;0;0;False;0;0.3607843,1,0.3176471,1;0.9843137,0.572549,0.2039216,1;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.RangedFloatNode;437;2565.891,-522.5626;Inherit;False;Property;_DetailPower;Detail Power;7;0;Create;True;0;0;False;0;0.2;0.131;0;1;0;1;FLOAT;0
Node;AmplifyShaderEditor.RangedFloatNode;156;2583.597,-614.0981;Float;False;Property;_NormalPower;Normal Power;5;0;Create;True;0;0;False;0;0.5;1;0;1;0;1;FLOAT;0
Node;AmplifyShaderEditor.StaticSwitch;167;2256.74,-416.9143;Inherit;False;Property;_Usetint;Use tint;1;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;FLOAT;0;False;0;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;0;False;4;FLOAT;0;False;5;FLOAT;0;False;6;FLOAT;0;False;7;FLOAT;0;False;8;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.SimpleContrastOpNode;511;3382.388,-2194.524;Inherit;True;2;1;COLOR;0,0,0,0;False;0;FLOAT;0.5;False;1;COLOR;0
Node;AmplifyShaderEditor.BlendOpsNode;373;3054.351,-2462.558;Inherit;True;VividLight;False;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;FLOAT;1;False;1;COLOR;0
Node;AmplifyShaderEditor.SamplerNode;412;2879.598,-1408.369;Inherit;True;Property;_DetailTexture;Detail Texture;18;0;Create;True;0;0;False;0;-1;None;16d574e53541bba44a84052fa38778df;True;0;False;white;Auto;False;Object;-1;Auto;Texture2D;6;0;SAMPLER2D;;False;1;FLOAT2;0,0;False;2;FLOAT;0;False;3;FLOAT2;0,0;False;4;FLOAT2;0,0;False;5;FLOAT;1;False;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.SamplerNode;155;2558.487,-1312.805;Inherit;True;Property;_NormalTexture;Normal Texture;12;0;Create;True;0;0;False;0;-1;None;a24ca7f1e51054ac39d73b6afd634938;True;0;True;bump;Auto;True;Object;-1;Auto;Texture2D;6;0;SAMPLER2D;0,0;False;1;FLOAT2;1,0;False;2;FLOAT;1;False;3;FLOAT2;0,0;False;4;FLOAT2;0,0;False;5;FLOAT;1;False;5;FLOAT3;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.LerpOp;470;2880.248,-1198.262;Inherit;True;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.LerpOp;372;3647.948,-2500.964;Inherit;True;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0.5019608,0.5019608,0.5019608,1;False;1;COLOR;0
Node;AmplifyShaderEditor.StaticSwitch;421;2881.954,-426.3662;Inherit;False;Property;_Usedetail;Use detail;3;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;FLOAT;0;False;0;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;0;False;4;FLOAT;0;False;5;FLOAT;0;False;6;FLOAT;0;False;7;FLOAT;0;False;8;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.LerpOp;522;2876.863,-673.5438;Inherit;False;3;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.RangedFloatNode;152;2860.833,-526.5441;Inherit;False;Property;_TintPower;Tint Power;4;0;Create;True;0;0;False;0;0.2;0.2;0;1;0;1;FLOAT;0
Node;AmplifyShaderEditor.StaticSwitch;303;4045.159,-2514.276;Inherit;False;Property;_ShadowMap;ShadowMap;13;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;COLOR;0,0,0,0;False;0;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;3;COLOR;0,0,0,0;False;4;COLOR;0,0,0,0;False;5;COLOR;0,0,0,0;False;6;COLOR;0,0,0,0;False;7;COLOR;0,0,0,0;False;8;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.ColorNode;112;3152.841,-950.944;Inherit;False;Property;_TintColor;Tint Color;9;0;Create;True;0;0;False;0;0.5019608,0.5019608,0.5019608,0;0.5019608,0.5019608,0.5019608,0;True;0;5;COLOR;0;FLOAT;1;FLOAT;2;FLOAT;3;FLOAT;4
Node;AmplifyShaderEditor.LerpOp;521;3206.374,-1263.545;Inherit;False;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.LerpOp;529;3105.42,-753.1784;Inherit;False;3;0;FLOAT;0;False;1;FLOAT;0;False;2;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.StaticSwitch;316;3104.51,-443.896;Inherit;False;Property;_Background;Background;19;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;FLOAT;0;False;0;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;0;False;4;FLOAT;0;False;5;FLOAT;0;False;6;FLOAT;0;False;7;FLOAT;0;False;8;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.StaticSwitch;509;4309.204,-2515.997;Inherit;False;Property;_RealtimeShadows;Realtime Shadows;16;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;COLOR;0,0,0,0;False;0;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;3;COLOR;0,0,0,0;False;4;COLOR;0,0,0,0;False;5;COLOR;0,0,0,0;False;6;COLOR;0,0,0,0;False;7;COLOR;0,0,0,0;False;8;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.StaticSwitch;196;3424.759,-469.6624;Inherit;False;Property;_Usenormal;Use normal;2;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;FLOAT;0;False;0;FLOAT;0;False;2;FLOAT;0;False;3;FLOAT;0;False;4;FLOAT;0;False;5;FLOAT;0;False;6;FLOAT;0;False;7;FLOAT;0;False;8;FLOAT;0;False;1;FLOAT;0
Node;AmplifyShaderEditor.BlendOpsNode;516;3404.196,-884.9404;Inherit;False;ColorBurn;True;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;FLOAT;1;False;1;COLOR;0
Node;AmplifyShaderEditor.BlendOpsNode;517;3901.638,-889.3171;Inherit;False;ColorBurn;True;3;0;COLOR;0,0,0,0;False;1;FLOAT;0;False;2;FLOAT;1;False;1;COLOR;0
Node;AmplifyShaderEditor.StaticSwitch;508;4629.176,-2518.715;Inherit;False;Property;_Shadows;Shadows;15;0;Create;True;0;0;False;0;1;0;0;True;;Toggle;2;Key0;Key1;Create;True;9;1;COLOR;0,0,0,0;False;0;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;3;COLOR;0,0,0,0;False;4;COLOR;0,0,0,0;False;5;COLOR;0,0,0,0;False;6;COLOR;0,0,0,0;False;7;COLOR;0,0,0,0;False;8;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.LerpOp;526;5071.47,-2054.064;Inherit;False;3;0;COLOR;0,0,0,0;False;1;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.StaticSwitch;525;5352.185,-2245.461;Inherit;False;Property;_Demo;Demo;14;0;Create;True;0;0;False;0;0;1;1;True;;Toggle;2;Key0;Key1;Create;True;9;1;COLOR;0,0,0,0;False;0;COLOR;0,0,0,0;False;2;COLOR;0,0,0,0;False;3;COLOR;0,0,0,0;False;4;COLOR;0,0,0,0;False;5;COLOR;0,0,0,0;False;6;COLOR;0,0,0,0;False;7;COLOR;0,0,0,0;False;8;COLOR;0,0,0,0;False;1;COLOR;0
Node;AmplifyShaderEditor.StandardSurfaceOutputNode;449;5669.326,-2346.986;Half;False;True;-1;2;HSLightcapDemo_Editor;0;0;CustomLighting;HoneyShibari/HSLightcap_Standard_Demo;False;False;False;False;True;True;True;True;True;False;True;True;False;False;True;False;False;False;False;False;False;Back;0;False;-1;0;False;-1;False;0;False;-1;0;False;-1;False;0;Opaque;0.5;True;True;0;False;Opaque;;Geometry;All;14;all;True;True;True;True;0;False;-1;False;0;False;-1;255;False;-1;255;False;-1;0;False;-1;0;False;-1;0;False;-1;0;False;-1;0;False;-1;0;False;-1;0;False;-1;0;False;-1;False;2;15;10;25;False;0.5;True;0;0;False;-1;0;False;-1;0;0;False;-1;0;False;-1;0;False;-1;0;False;-1;0;False;0;0,0,0,0;VertexOffset;True;False;Cylindrical;False;Relative;0;;-1;-1;-1;-1;0;False;0;0;False;-1;-1;0;False;-1;0;0;0;False;0.1;False;-1;0;False;-1;15;0;FLOAT3;0,0,0;False;1;FLOAT3;0,0,0;False;2;FLOAT3;0,0,0;False;3;FLOAT3;0,0,0;False;4;FLOAT;0;False;6;FLOAT3;0,0,0;False;7;FLOAT3;0,0,0;False;8;FLOAT;0;False;9;FLOAT;0;False;10;FLOAT;0;False;13;FLOAT3;0,0,0;False;11;FLOAT3;0,0,0;False;12;FLOAT3;0,0,0;False;14;FLOAT4;0,0,0,0;False;15;FLOAT3;0,0,0;False;0
Node;AmplifyShaderEditor.CommentaryNode;190;53.9492,-3854.579;Inherit;False;1046.674;752.4886;;3;192;191;189;Flat or Tex;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;450;50.3911,-1665.911;Inherit;False;1803.786;1184.38;;0;Custom Colors;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;343;1475.833,-3937.931;Inherit;False;437.4443;179.4399;;0;Background;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;189;89.41392,-3789.168;Inherit;False;255.0925;256.7981;;0;Tint;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;423;1484.867,-3603.207;Inherit;False;829.579;573.1687;;0;Detail?;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;515;4894.065,-3588.592;Inherit;False;1398.858;354.983;;0;Dynamic Lights;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;166;2482.534,-3613.291;Inherit;False;1377.823;518.8007;;0;Normal;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;191;498.5728,-3479.517;Inherit;False;551.7404;320.0676;Base color becomes TextureTint;0;Use Tint on Tex?;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;192;593.8369,-3784.886;Inherit;False;304.6045;256.0225;;0;Texture?;1,1,1,1;0;0
Node;AmplifyShaderEditor.CommentaryNode;304;4021.789,-3602.22;Inherit;False;760.7012;436.27;;0;Shaodowmap;1,1,1,1;0;0
WireConnection;49;0;47;0
WireConnection;49;1;48;0
WireConnection;321;0;49;0
WireConnection;321;1;50;0
WireConnection;228;0;227;0
WireConnection;326;0;321;0
WireConnection;326;1;50;0
WireConnection;254;0;326;0
WireConnection;254;2;228;0
WireConnection;226;1;254;0
WireConnection;510;0;226;0
WireConnection;513;1;510;0
WireConnection;461;0;456;0
WireConnection;461;1;455;0
WireConnection;416;1;226;0
WireConnection;416;0;226;0
WireConnection;168;1;530;0
WireConnection;168;0;106;0
WireConnection;464;0;461;0
WireConnection;464;1;462;0
WireConnection;514;1;513;0
WireConnection;514;3;512;0
WireConnection;469;0;464;0
WireConnection;469;1;463;0
WireConnection;151;0;416;0
WireConnection;151;1;168;0
WireConnection;151;2;195;0
WireConnection;511;1;514;0
WireConnection;373;0;416;0
WireConnection;373;1;168;0
WireConnection;373;2;195;0
WireConnection;470;0;469;0
WireConnection;470;1;468;0
WireConnection;470;2;295;0
WireConnection;372;0;151;0
WireConnection;372;1;373;0
WireConnection;372;2;511;0
WireConnection;421;1;167;0
WireConnection;421;0;167;0
WireConnection;522;0;451;0
WireConnection;522;1;156;0
WireConnection;522;2;437;0
WireConnection;303;1;372;0
WireConnection;303;0;372;0
WireConnection;521;0;155;0
WireConnection;521;1;470;0
WireConnection;521;2;412;0
WireConnection;529;0;522;0
WireConnection;529;1;152;0
WireConnection;316;0;421;0
WireConnection;509;1;303;0
WireConnection;509;0;303;0
WireConnection;196;1;316;0
WireConnection;196;0;316;0
WireConnection;516;0;112;0
WireConnection;516;1;521;0
WireConnection;516;2;529;0
WireConnection;517;0;516;0
WireConnection;517;1;196;0
WireConnection;508;1;509;0
WireConnection;508;0;509;0
WireConnection;526;0;508;0
WireConnection;526;1;517;0
WireConnection;525;1;526;0
WireConnection;525;0;508;0
WireConnection;449;13;525;0
ASEEND*/
//CHKSM=9DE853BEECFE662F8B2D2E9F79D35512B8B3D5A0