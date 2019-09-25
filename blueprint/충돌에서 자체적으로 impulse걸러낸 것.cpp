# include <SDL.h>
# include <SDL_image.h>
# include <SDL_opengl.h>
# include <stdio.h>
# include <float.h>
# include <stdlib.h>
# include <math.h>

#pragma comment (lib,"SDL2")
#pragma comment (lib,"SDL2main")
#pragma comment (lib,"SDL2_image")
#pragma comment (lib, "opengl32.lib")

# define Length 40.0 // lenght of one block
# define fpss 60.0
# define g (9.8)
# define pi 3.14159265
# define shocknum 5
# define warmth 1.0

# define threshold 0.01
# define theta_slop 0.01
# define slop 0.01


struct material {
	double restitution;
	double density;
	double us; // u static constant
	double uk; // u kinetic constant
	char number; // identify number of material
}; typedef struct material Material;

struct shape {
	int vertex_num; // number of vertex on shape
	double ** vertex; // vertex vector from center of mass
	int * vertex_contact_index;
	bool * vertex_contact;
	double ** normal; // normal vector of polygon
	double volume;
	double inertia_constant; // I/(m*V) value
	int number; // identify number of shape
	double max_distance;

}; typedef struct shape Shape;

struct object {
	Shape * shape; // shape of object

	double x[2]; // position vector x of COM
	double v[2]; // velocity vector v of COM
	double a[2]; // acceleration vector a of COM

	double th; // angle value theta to COM
	double w; // angular velocity w to COM
	double alp; // angular acceleration alpha to COM

	double F[2]; // force on object of COM

	double inv_mass; // inverse of mass
	double inv_inertia; // inverse of inertial

	double delw;
	double delv[2];

	double past_delw;
	double past_delv[2];

	bool stable;

	Material material;

}; typedef struct object Object;

struct contact_information {
	bool run;
	int numA;
	int numB;
	int indexA;
	int indexB;
	double rA[2]; // position vector from center of Mass A
	double rB[2]; // position vector from center of Mass B
	double impulse[2];
	double depth;
}; typedef struct contact_information cinf;

void cycle(Object A[], int n, int NumberOfObject, cinf contact[]);
void one_term(Object A[], int NumberOfObject, cinf contact[]);
void set_shape(Shape * shape, int n);
void force_zero(Object * A);
void find_fraction(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB);
void find_impulse(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB);
void gravity(Object * A);
void movement(Object * A);
void positionalcorrection(Object * A, Object * B);
void set_material(Object * object, int n);
void set_object(Object * A, int shape, int material, double x1, double x2, double v1, double v2, double a1, double a2, double th, double w, double alp, double F1, double F2);
void set_mass(Object * object);

void v_sum(double v1[2], double v2[2], double v_out[2]);

void v_sub(double v1[2], double v2[2], double v_out[2]);

double v_dot(double v1[2], double v2[2]);

double v_cross(double v1[2], double v2[2]);

void v_cross_k(double vec[2], double k, double v_out[2]);

void v_cross_k_inv(double k, double vec[2], double v_out[2]);

double v_magnitude(double v[2]);

void v_normalization(double v_in[2], double v_out[2]);

void v_normalization_s(double v_in[2]);

void mat_v_product(double mat[2][2], double v[2], double v_out[2]);

void set_rot_mat(double mat[2][2], double radius);

void find_normal_v(double v[2], double dir[2], double v_out[2]);

void find_perpend_v(double v[2], double dir[2], double v_out[2]);

void v_mult(double v[2], double n, double v_out[2]);

double minimum(double a, double b);

double maximum(double a, double b);

double absolute(double a);

void draw(Object A[], int NumberOfObject);

double click_perception(int mouse_x, int mouse_y, Object * A);

void reassign_vertex(Object * A);

void delta_zero(Object * A);

void shock_propagation(Object A[], int NumberOfObject);

void stability_zero(Object * A);

void find_shock(Object * A, Object * B, bool * contact);

void collision_check(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB);

void uncollide_check(Object A[], int NumberOfObject, cinf contact[]);

void warmstarting(Object A[], int NumberOfObject, cinf contact[]);


SDL_Window * window;
SDL_Renderer * renderer;

SDL_Texture *texture[4];


SDL_Texture * loadTexture(const char *file) {
	SDL_Surface * surface;
	SDL_Texture *texture;

	surface = IMG_Load(file);
	if (surface == NULL) {
		printf("%s파일을 읽을 수 없습니다.\n", file);
		return NULL;
	}
	texture = SDL_CreateTextureFromSurface(renderer, surface);
	if (texture == NULL) {
		printf("텍스쳐를 생성할 수 없습니다.\n");
	}
	SDL_FreeSurface(surface);
	return texture;
}

void stretchTextureEx(SDL_Renderer *renderer, int x, int y, int w, int h, SDL_Texture *texture, float angle, SDL_RendererFlip flip = SDL_FLIP_NONE) {
	SDL_Rect src, dst;
	SDL_Point center;

	src.x = src.y = 0;
	SDL_QueryTexture(texture, NULL, NULL, &src.w, &src.h);

	dst.x = x;
	dst.y = y;
	dst.w = w;
	dst.h = h;

	center.x = w / 2;
	center.y = h / 2;

	SDL_RenderCopyEx(renderer, texture, &src, &dst, angle, &center, flip);
}

void drawTexture(SDL_Renderer *renderer, int x, int y, SDL_Texture *texture) {
	SDL_Rect src, dst;

	src.x = src.y = 0;
	SDL_QueryTexture(texture, NULL, NULL, &src.w, &src.h);

	dst.x = x;
	dst.y = y;
	dst.w = src.w;
	dst.h = src.h;

	SDL_RenderCopy(renderer, texture, &src, &dst);
}


int main(int argc, char **argv) {
	SDL_Init(SDL_INIT_VIDEO);
	window = SDL_CreateWindow("Blueprint", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1360, 768, SDL_RENDERER_ACCELERATED);
	renderer = SDL_CreateRenderer(window, SDL_VIDEO_RENDER_OGL, 0);

	SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

	bool quit = false;
	bool main_texture = false;
	bool stage_texture = false;
	bool running_texture = false;
	bool startbutton = false; bool quitbutton = false; bool optionbutton = false;
	bool backbutton = false; bool nextbutton = false; bool mainmenubutton = false;
	bool step[8] = { false,false,false ,false ,false ,false ,false ,false };
	char stage_number = 0; char step_number = 0;

	SDL_Event event;
	SDL_Texture * mainmenu;
	SDL_Texture * arrow;

	SDL_Texture * stage;
	SDL_Texture * back; // back button
	SDL_Texture * next; // next button
	SDL_Texture * mainmenu_button; // mainmenu button
	SDL_Texture * stage_menu; // stage title

	SDL_Texture * map;
	SDL_Texture * back_to_stage;
	SDL_Texture * description;
	SDL_Texture * start;
	SDL_Texture * right_menu;
	SDL_Texture * large_soil;
	SDL_Texture * wood_one_block;
	SDL_Texture * wood_two_block;
	SDL_Texture * wood_three_block;
	SDL_Texture * wood_triangle_block;

	enum Scene {main_menu=1, stage_select1, stage_select2, stage_select3, stage_select4, stage_select5, in_stage, option};
	Scene scene = main_menu;

	while (!quit) {
		if (scene == main_menu) {

			if (main_texture == false) {
				mainmenu = loadTexture("resource\\main_menu.png");
				arrow = loadTexture("resource\\arrow.png");
				main_texture = true;
			}

			stretchTextureEx(renderer, 0, 0, 1360, 768, mainmenu, 0);

			if (SDL_PollEvent(&event)) {
				switch (event.type) {
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_LEFT) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 1111 && x <= 1267 && y >= 368 && y <= 415) { // click start button
							scene = stage_select1;
							main_texture = false;
							SDL_DestroyTexture(mainmenu);
							SDL_DestroyTexture(arrow);
							startbutton = false;
						}
						else if (x >= 1067 && x <= 1267 && y >= 433 && y <= 498) { // click option button
							scene = option;
							main_texture = false;
							SDL_DestroyTexture(mainmenu);
							SDL_DestroyTexture(arrow);
						}
						else if (x >= 1121 && x <= 1267 && y >= 516 && y <= 589) { // click quit button
							quit = true;
							main_texture = false;
							SDL_DestroyTexture(mainmenu);
							SDL_DestroyTexture(arrow);
						}
					}
					break;
				case SDL_MOUSEMOTION :
						int x, y;
						x = event.motion.x;
						y = event.motion.y;

						if (x >= 1111 && x <= 1267 && y >= 368 && y <= 415) { // on start button
							startbutton = true;
						}
						else if (x >= 1067 && x <= 1267 && y >= 433 && y <= 498) { // on option button
							optionbutton = true;
						}
						else if (x >= 1121 && x <= 1267 && y >= 516 && y <= 589) { // on quit button
							quitbutton = true;
						}
						else {
							startbutton = false;
							quitbutton = false;
							optionbutton = false;
						}
					break;
				}
			}
			if (startbutton == true) { // on start button
				stretchTextureEx(renderer, 1046, 359, 65, 65, arrow, 0);
			}
			else if (optionbutton == true) { // on option button
				stretchTextureEx(renderer, 1002, 433, 65, 65, arrow, 0);
			}
			else if (quitbutton == true) { // on quit button
				stretchTextureEx(renderer, 1056, 520, 65, 65, arrow, 0);
			}
		}
		else if (scene == stage_select1|| scene == stage_select2|| scene == stage_select3|| scene == stage_select4|| scene == stage_select5) {

			if (stage_texture == false) {
				stage = loadTexture("resource\\stage_basic.png");
				arrow = loadTexture("resource\\arrow.png");
				back=loadTexture("resource\\back.png");
				next= loadTexture("resource\\next.png");
				mainmenu_button= loadTexture("resource\\mainmenu.png");
				stage_menu= loadTexture("resource\\stage1menu.png");
				stage_texture = true;
			}

			stretchTextureEx(renderer, 0, 0, 1360, 768, stage, 0);
			drawTexture(renderer, 556, 56, stage_menu);
			drawTexture(renderer, 975, 669, mainmenu_button);
			if (scene != stage_select5) drawTexture(renderer, 1135, 57, next);
			if (scene != stage_select1) drawTexture(renderer, 85, 57, back);

			if (SDL_PollEvent(&event)) {
				switch (event.type) {
				case SDL_QUIT:
					quit = true;
					break;
				case SDL_MOUSEBUTTONDOWN:
					if (event.button.button == SDL_BUTTON_LEFT) {
						int x, y;
						x = event.motion.x;
						y = event.motion.y;
						if (x >= 85 && x <= 246 && y >= 57 && y <= 117 && scene != stage_select1) { // click back
							if (scene == stage_select5) scene = stage_select4;
							else if (scene == stage_select4) scene = stage_select3;
							else if (scene == stage_select3) scene = stage_select2;
							else if (scene == stage_select2) scene = stage_select1;
							if (scene == stage_select1) { stage_menu = loadTexture("resource\\stage1menu.png"); backbutton = false; }
							else if (scene == stage_select2) stage_menu = loadTexture("resource\\stage2menu.png");
							else if (scene == stage_select3) stage_menu = loadTexture("resource\\stage3menu.png");
							else if (scene == stage_select4) stage_menu = loadTexture("resource\\stage4menu.png");
							else if (scene == stage_select5) stage_menu = loadTexture("resource\\stage5menu.png");
						}
						else if (x >= 1135 && x <= 1293 && y >= 57 && y <= 119 && scene != stage_select5) { // click next
							if (scene == stage_select1) scene = stage_select2;
							else if (scene == stage_select2) scene = stage_select3;
							else if (scene == stage_select3) scene = stage_select4;
							else if (scene == stage_select4) scene = stage_select5;
							if (scene == stage_select1) stage_menu = loadTexture("resource\\stage1menu.png");
							else if (scene == stage_select2) stage_menu = loadTexture("resource\\stage2menu.png");
							else if (scene == stage_select3) stage_menu = loadTexture("resource\\stage3menu.png");
							else if (scene == stage_select4) stage_menu = loadTexture("resource\\stage4menu.png");
							else if (scene == stage_select5) { stage_menu = loadTexture("resource\\stage5menu.png"); nextbutton = false; }
						}
						else if (x >= 975 && x <= 1322 && y >= 669 && y <= 740) { // click mainmenu button
							stage_texture = false;
							mainmenubutton = false;
							scene = main_menu;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
						}
						else if (x >= 105 && x <= 211 && y >= 144 && y <= 261) { // click step1 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 1;
						}
						else if (x >= 383 && x <= 538 && y >= 141 && y <= 287) { // click step2 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 2;
						}
						else if (x >= 699 && x <= 899 && y >= 141 && y <= 315) { // click step3 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 3;
						}
						else if (x >= 1020 && x <= 1269 && y >= 141 && y <= 344) { // click step4 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 4;
						}
						else if (x >= 106 && x <= 260 && y >= 414 && y <= 590) { // click step5 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 5;
						}
						else if (x >= 370 && x <= 572 && y >= 414 && y <= 619) { // click step6 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 6;
						}
						else if (x >= 687 && x <= 937 && y >= 445 && y <= 592) { // click step7 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 7;
						}
						else if (x >= 1019 && x <= 1271 && y >= 414 && y <= 618) { // click step8 button
							stage_texture = false;
							step[0] = false;
							scene = in_stage;
							SDL_DestroyTexture(stage);
							SDL_DestroyTexture(arrow);
							SDL_DestroyTexture(back);
							SDL_DestroyTexture(next);
							SDL_DestroyTexture(mainmenu_button);
							SDL_DestroyTexture(stage_menu);
							stage_number = scene - 1;
							step_number = 8;
						}
					}
					break;
				case SDL_MOUSEMOTION:
					int x, y;
					x = event.motion.x;
					y = event.motion.y;

					if (x >= 85 && x <= 246 && y >= 57 && y <= 117 && scene != stage_select1) { // on back button
						backbutton = true;
					}
					else if (x >= 1135 && x <= 1293 && y >= 57 && y <= 119 && scene != stage_select5) { // on next button
						nextbutton = true;
					}
					else if (x >= 975 && x <= 1322 && y >= 669 && y <= 740) { // on mainmenu button
						mainmenubutton = true;
					}
					else if (x >= 105 && x <= 211 && y >= 144 && y <= 261) { // on step1 button
						step[0]= true;
					}
					else if (x >= 383 && x <= 538 && y >= 141 && y <= 287) { // on step2 button
						step[1] = true;
					}
					else if (x >= 699 && x <= 899 && y >= 141 && y <= 315) { // on step3 button
						step[2] = true;
					}
					else if (x >= 1020 && x <= 1269 && y >= 141 && y <= 344) { // on step4 button
						step[3] = true;
					}
					else if (x >= 106 && x <= 260 && y >= 414 && y <= 590) { // on step5 button
						step[4] = true;
					}
					else if (x >= 370 && x <= 572 && y >= 414 && y <= 619) { // on step6 button
						step[5] = true;
					}
					else if (x >= 687 && x <= 937 && y >= 445 && y <= 592) { // on step7 button
						step[6] = true;
					}
					else if (x >= 1019 && x <= 1271 && y >= 414 && y <= 618) { // on step8 button
						step[7] = true;
					}
					else {
						backbutton= false;
						nextbutton = false;
						mainmenubutton = false;
						step[0] = false;
						step[1] = false;
						step[2] = false;
						step[3] = false;
						step[4] = false;
						step[5] = false;
						step[6] = false;
						step[7] = false;
					}
					break;
				}
			}
			if (backbutton == true) { // on back button
				stretchTextureEx(renderer, 20, 55, 65, 65, arrow, 0);
			}
			else if (nextbutton == true) { // on next button
				stretchTextureEx(renderer, 1070, 55, 65, 65, arrow, 0);
			}
			else if (mainmenubutton == true) { // on mainmenu button
				stretchTextureEx(renderer, 910, 667, 65, 65, arrow, 0);
			}
			else if (step[0] == true) { // on step1 button
				stretchTextureEx(renderer, 123, 304, 65, 65, arrow, -90);
			}
			else if (step[1] == true) { // on step2 button
				stretchTextureEx(renderer, 431, 304, 65, 65, arrow, -90);
			}
			else if (step[2] == true) { // on step3 button
				stretchTextureEx(renderer, 759, 304, 65, 65, arrow, -90);
			}
			else if (step[3] == true) { // on step4 button
				stretchTextureEx(renderer, 1108, 304, 65, 65, arrow, -90);
			}
			else if (step[4] == true) { // on step5 button
				stretchTextureEx(renderer, 123, 610, 65, 65, arrow, -90);
			}
			else if (step[5] == true) { // on step6 button
				stretchTextureEx(renderer, 431, 610, 65, 65, arrow, -90);
			}
			else if (step[6] == true) { // on step7 button
				stretchTextureEx(renderer, 759, 610, 65, 65, arrow, -90);
			}
			else if (step[7] == true) { // on step8 button
				stretchTextureEx(renderer, 1108, 610, 65, 65, arrow, -90);
			}

		}
		else if (scene == in_stage) {

			Object * object;
			Object * tempt_object;
			cinf * contact;
			double angle = 0;
			int population = 0;
			int material = 2; // wood
			int click_x, click_y;
			bool stage_prepare = true;
			bool stage_start = false;
			bool back_to_stage_button = false;
			bool click_one_block = false;
			bool click_two_block = false;
			bool click_three_block = false;
			bool click_triangle_block = false;
			char click_block = -1; // index of block

			int i;

			map = loadTexture("resource\\stage1_back.png");
			right_menu = loadTexture("resource\\stage_menu.png");
			back_to_stage = loadTexture("resource\\back_to_stage.png");
			description = loadTexture("resource\\running_description.png");
			start = loadTexture("resource\\start!.png");
			arrow= loadTexture("resource\\arrow.png");
			wood_one_block = loadTexture("resource\\wood_one.png");
			wood_two_block = loadTexture("resource\\wood_two.png");
			wood_three_block = loadTexture("resource\\wood_three.png");
			wood_triangle_block = loadTexture("resource\\wood_triangle.png");
			large_soil= loadTexture("resource\\two_soil.png");

			population = 1;
			object = (Object*)malloc(sizeof(Object));
			set_object(&object[0], 99, 99, 536, 727, 0, 0, 0, 0, 0, 0, 0, 0, 0);

			while (stage_prepare == true && quit == false) {
				drawTexture(renderer, 0, 0, map);
				drawTexture(renderer, 1004, 0, right_menu);
				drawTexture(renderer, 1108, 709, back_to_stage);
				drawTexture(renderer, 1043, 51, description);
				drawTexture(renderer, 1088, 622, start);
				stretchTextureEx(renderer, 1124, 408,Length,Length, wood_one_block,angle);
				stretchTextureEx(renderer, 1234, 408, 2*Length, Length, wood_two_block, angle);
				stretchTextureEx(renderer, 1084, 524, 3*Length, Length, wood_three_block, angle);
				stretchTextureEx(renderer, 1255, 529, Length, Length*sqrt(3)*0.5, wood_triangle_block, angle);
			

				int x, y;
				x = event.motion.x;
				y = event.motion.y;

				int mouse_x, mouse_y;

				if (SDL_PollEvent(&event)) {
						if(event.type==SDL_QUIT) quit = true;

						if (event.type == SDL_MOUSEBUTTONDOWN) {
							if (event.button.button == SDL_BUTTON_LEFT) {
								int x, y;
								x = event.motion.x;
								y = event.motion.y;
								if (x >= 1108 && x <= 1355 && y >= 709 && y <= 736) { // click back to stage select
									stage_prepare = false;
									SDL_DestroyTexture(map);
									SDL_DestroyTexture(back_to_stage);
									SDL_DestroyTexture(description);
									SDL_DestroyTexture(start);
									SDL_DestroyTexture(right_menu);
									SDL_DestroyTexture(wood_one_block);
									SDL_DestroyTexture(wood_two_block);
									SDL_DestroyTexture(wood_three_block);
									SDL_DestroyTexture(wood_triangle_block);
									scene = stage_select1;
								}
								else if (x >= 1088 && x <= 1307 && y >= 622 && y <= 670) { // click start button
									stage_start = true;
									stage_prepare = false;
		//							SDL_DestroyTexture(map);
									SDL_DestroyTexture(back_to_stage);
									SDL_DestroyTexture(description);
									SDL_DestroyTexture(start);
		//							SDL_DestroyTexture(right_menu);
		//							SDL_DestroyTexture(wood_one_block);
		//							SDL_DestroyTexture(wood_two_block);
		//							SDL_DestroyTexture(wood_three_block);
		//							SDL_DestroyTexture(wood_triangle_block);
									contact = (cinf *)malloc(sizeof(cinf) * 4 * population);
									for (i = 0; i < 4 * population; i++) {
										contact[i].run = false;
										contact[i].impulse[0] = 0;
										contact[i].impulse[1] = 0;
									}
								}
								else if (x >= 1124 && x <= 1124 + Length && y >= 408 && y <= 408 + Length) { // click one block
									click_one_block = true;
								}
								else if (x >= 1234 && x <= 1234 + 2 * Length && y >= 408 && y <= 408 + Length) { // click two block
									click_two_block = true;
								}
								else if (x >= 1084 && x <= 1084 + 3 * Length && y >= 524 && y <= 524 + Length) { // click three block
									click_three_block = true;
								}
								else if (x >= 1255 && x <= 1255 + Length && y >= 529 && y <= 529 + Length*sqrt(3)*0.5) { // click triangle block
									click_triangle_block = true;
								}
								else {
									for (i = 0; i < population; i++) if (click_perception(mouse_x, mouse_y, &object[i]) < 0) {
										click_block = i; click_x = mouse_x; click_y = mouse_y;
									}
								}
							}
						}

						if (event.type == SDL_MOUSEMOTION) {
							int x, y;
							x = event.motion.x;
							y = event.motion.y;
							mouse_x = x;
							mouse_y = y;

							if (x >= 1108 && x <= 1355 && y >= 709 && y <= 736) { // on back to stage button
								back_to_stage_button = true;
							}
							else {
								back_to_stage_button = false;
							}
							if(click_block!=-1) {
								object[click_block].x[0] = object[click_block].x[0] + (mouse_x - click_x);
								object[click_block].x[1] = object[click_block].x[1] + (mouse_y - click_y);
								click_x = mouse_x; click_y = mouse_y;
							}
						}

						if (event.type == SDL_MOUSEBUTTONUP) {
							if (click_one_block == true|| click_two_block == true|| click_three_block == true || click_triangle_block == true) {
								if (mouse_x < 1004) {
									int i, j;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&tempt_object[i], object[i].shape->number, object[i].material.number, object[i].x[0], object[i].x[1], 0, 0, 0, 0, object[i].th, 0, 0, 0, 0);
									if (population != 0) {
										for (i = 0; i < population; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
											free(object[i].shape->vertex); free(object[i].shape->normal);
											free(object[i].shape->vertex_contact_index);
											free(object[i].shape->vertex_contact);
											free(object[i].shape);
										}
										free(object);
									}
									population = population + 1;
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population - 1; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0], tempt_object[i].x[1], 0, 0, 0, 0, tempt_object[i].th, 0, 0, 0, 0);
									if (click_one_block == true) {
										click_one_block = false;
										set_object(&object[population-1], 1, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi/180, 0, 0, 0, 0);
									}
									else if (click_two_block == true) {
										click_two_block = false;
										set_object(&object[population - 1], 2, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									}
									else if (click_three_block == true) {
										click_three_block = false;
										set_object(&object[population - 1], 3, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									}
									else if (click_triangle_block == true) {
										click_triangle_block = false;
										set_object(&object[population - 1], 4, material, mouse_x, mouse_y, 0, 0, 0, 0, angle*pi / 180, 0, 0, 0, 0);
									}
									if (population-1 != 0) {
										for (i = 0; i < population-1; i++) {
											for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
											free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
											free(tempt_object[i].shape->vertex_contact_index);
											free(tempt_object[i].shape->vertex_contact);
											free(tempt_object[i].shape);
										}
										free(tempt_object);
									}
									for (i = 0; i < population;i++) reassign_vertex(&object[i]);
								}
								else if (mouse_x >= 1004) {
									click_one_block = false;
									click_two_block = false;
									click_three_block = false;
									click_triangle_block = false;
								}
							}
							if (click_block != -1) {
								if (object[click_block].x[0] >= 1004) {
									int i, j;
									population = population - 1;
									tempt_object = (Object *)malloc(sizeof(Object)*population);
									i = 0; j = 0;
									for (i = 0; i < population + 1; i++) if (i != click_block) {
										set_object(&tempt_object[j], object[i].shape->number, object[i].material.number, object[i].x[0], object[i].x[1], 0, 0, 0, 0, object[i].th, 0, 0, 0, 0);
										j = j + 1;
									}
									for (i = 0; i < population + 1; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(object[i].shape->vertex[j]); free(object[i].shape->normal[j]); }
										free(object[i].shape->vertex); free(object[i].shape->normal);
										free(object[i].shape->vertex_contact_index);
										free(object[i].shape->vertex_contact);
										free(object[i].shape);
									}
									free(object);
									object = (Object *)malloc(sizeof(Object)*population);
									for (i = 0; i < population; i++) set_object(&object[i], tempt_object[i].shape->number, tempt_object[i].material.number, tempt_object[i].x[0], tempt_object[i].x[1], 0, 0, 0, 0, tempt_object[i].th, 0, 0, 0, 0);
									for (i = 0; i < population; i++) {
										for (j = 0; j < object[i].shape->vertex_num; j++) { free(tempt_object[i].shape->vertex[j]); free(tempt_object[i].shape->normal[j]); }
										free(tempt_object[i].shape->vertex); free(tempt_object[i].shape->normal);
										free(tempt_object[i].shape->vertex_contact_index);
										free(tempt_object[i].shape->vertex_contact);
										free(tempt_object[i].shape);
									}
									free(tempt_object);
								}
								click_block = -1;
								for (i = 0; i < population; i++) reassign_vertex(&object[i]);
							}
						}

						if (event.type == SDL_KEYDOWN) {
							if (event.key.keysym.sym == SDLK_r) {
								angle = angle - 10;
								if (click_block != -1) object[click_block].th = object[click_block].th - 10 * pi / 180;
							}
							else if (event.key.keysym.sym == SDLK_t) {
								angle = angle + 10;
								if (click_block != -1) object[click_block].th = object[click_block].th + 10 * pi / 180;
							}
						}
				}
				if (back_to_stage_button == true) stretchTextureEx(renderer, 1081, 708, 27, 27, arrow, 0);

				if (click_one_block == true) stretchTextureEx(renderer, mouse_x - Length / 2, mouse_y - Length / 2, Length, Length, wood_one_block, angle);
				else if (click_two_block == true) stretchTextureEx(renderer, mouse_x - Length, mouse_y - Length / 2, 2 * Length, Length, wood_two_block, angle);
				else if (click_three_block == true) stretchTextureEx(renderer, mouse_x - 3 * Length / 2, mouse_y - Length / 2, 3 * Length, Length, wood_three_block, angle);
				else if (click_triangle_block == true) stretchTextureEx(renderer, mouse_x - Length / 2, mouse_y - sqrt(3)*0.5*Length / 2, Length, sqrt(3)*0.5*Length, wood_triangle_block, angle);

				for (i = 0; i < population; i++) {
					if (object[i].shape->number == 1) {
						stretchTextureEx(renderer, object[i].x[0] - Length / 2, object[i].x[1] - Length / 2, Length, Length, wood_one_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 2) {
						stretchTextureEx(renderer, object[i].x[0] - Length, object[i].x[1] - Length / 2, 2 * Length, Length, wood_two_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 3) {
						stretchTextureEx(renderer, object[i].x[0] - 3 * Length / 2, object[i].x[1] - Length / 2, 3 * Length, Length, wood_three_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 4) {
						stretchTextureEx(renderer, object[i].x[0] - Length / 2, object[i].x[1] - sqrt(3)*0.5*Length / 2, Length, sqrt(3)*0.5*Length, wood_triangle_block, object[i].th * 180 / pi);
					}
					else if (object[i].shape->number == 99) {
						stretchTextureEx(renderer, object[i].x[0] - 7 * Length, object[i].x[1] - 2*Length, 14 * Length, 4 * Length, large_soil, object[i].th * 180 / pi);
					}
				}

				SDL_RenderPresent(renderer);
				SDL_Delay(1);
				SDL_RenderClear(renderer);
			}
			if (stage_start == true) {
				while (1) {
//					printf("%f %f %f\n", object[2].v[0], object[2].w, object[2].th);
					if (SDL_PollEvent(&event)) {
						if (event.type == SDL_QUIT) quit = true;
						if (event.type == SDL_MOUSEBUTTONDOWN);
					}

					drawTexture(renderer, 0, 0, map);
					drawTexture(renderer, 1004, 0, right_menu);
					one_term(object, population, contact);
					for (i = 0; i < population; i++) {
						if (object[i].shape->number == 1) {
							stretchTextureEx(renderer, object[i].x[0] - Length / 2, object[i].x[1] - Length / 2, Length, Length, wood_one_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 2) {
							stretchTextureEx(renderer, object[i].x[0] - Length, object[i].x[1] - Length / 2, 2 * Length, Length, wood_two_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 3) {
							stretchTextureEx(renderer, object[i].x[0] - 3 * Length / 2, object[i].x[1] - Length / 2, 3 * Length, Length, wood_three_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 4) {
							stretchTextureEx(renderer, object[i].x[0] - Length / 2, object[i].x[1] - sqrt(3)*0.5*Length / 2, Length, sqrt(3)*0.5*Length, wood_triangle_block, object[i].th * 180 / pi);
						}
						else if (object[i].shape->number == 99) {
							stretchTextureEx(renderer, object[i].x[0] - 7 * Length, object[i].x[1] - 2*Length, 14 * Length, 4 * Length, large_soil, object[i].th * 180 / pi);
						}
					}
					SDL_RenderPresent(renderer);
					SDL_Delay(1);
					SDL_RenderClear(renderer);
				}
			}
		}
		SDL_RenderPresent(renderer);
		SDL_Delay(10);
		SDL_RenderClear(renderer);
	}







/*	while (!quit) {

		for (i = 0; i < number; i++) one_term(A, number);
		draw(A, number);

		if (SDL_PollEvent(&event)) {
			switch (event.type) {
			case SDL_QUIT:
					quit = true;
					break;
			}
		}
		printf("%f\n", A[1].th*180/pi);
		SDL_RenderClear(renderer);
		SDL_Delay(10);
	}

	SDL_Quit();
	*/
	return 0;

}

void draw(Object A[], int NumberOfObject) {

	stretchTextureEx(renderer, 0, 0, 500,500, texture[0], 0);

	int i;
	for (i = 0; i < NumberOfObject; i++) {
		if (A[i].shape->number == 1) {
			stretchTextureEx(renderer, A[i].x[0] - Length / 2, A[i].x[1] - Length / 2, Length, Length, texture[1], A[i].th * 180 / pi);
		}
		else if (A[i].shape->number == 2) {
			stretchTextureEx(renderer, A[i].x[0] - 3 * Length / 2, A[i].x[1] - Length / 2, 3 * Length,Length, texture[1], A[i].th * 180 / pi);
		}
		else if (A[i].shape->number == 99) {
			stretchTextureEx(renderer, A[i].x[0] - 9 * Length, A[i].x[1] - Length, 18 * Length, 2*Length, texture[2], A[i].th * 180 / pi);
		}
	}
		SDL_RenderPresent(renderer);
//		SDL_DestroyTexture(texture);
}

void cycle(Object A[], int n, int NumberOfObject, cinf contact[]) { // A[n]
	int i;

	for (i = 0; i < NumberOfObject; i++) if (i > n) if ((A[i].shape->max_distance + A[n].shape->max_distance)*(A[i].shape->max_distance + A[n].shape->max_distance)>(A[i].x[0] - A[n].x[0])*(A[i].x[0] - A[n].x[0]) + (A[i].x[1] - A[n].x[1])*(A[i].x[1] - A[n].x[1])) {
		find_impulse(&A[n], &A[i], NumberOfObject, contact, n, i);
		find_impulse(&A[i], &A[n], NumberOfObject, contact, i, n);
		find_fraction(&A[n], &A[i], NumberOfObject, contact, n, i);
		find_fraction(&A[i], &A[n], NumberOfObject, contact, i, n);
	}
//		for (i = 0; i < NumberOfObject; i++) A[i].delw = 0;
}

void one_term(Object A[], int NumberOfObject, cinf contact[]) {
	int i, j;

	for (i = 0; i < NumberOfObject; i++) {
		A[i].delv[0] = 0;
		A[i].delv[1] = 0;
		A[i].delw = 0;
	}

	for (i = 0; i < NumberOfObject; i++) { force_zero(&A[i]); gravity(&A[i]); }
	for (i = 0; i < NumberOfObject; i++) { movement(&A[i]); }

	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < NumberOfObject; j++) {
			if (i > j) { collision_check(&A[j], &A[i], NumberOfObject, contact, j, i); collision_check(&A[i], &A[j], NumberOfObject, contact, i, j); }
		}
	}

	uncollide_check(A, NumberOfObject, contact);

	warmstarting(A, NumberOfObject, contact);

	for (j = 0; j < 10;j++) for (i = 0; i < NumberOfObject; i++) cycle(A, i, NumberOfObject, contact);

		for (i = 0; i < NumberOfObject; i++) {
			A[i].v[0] = A[i].v[0] + A[i].delv[0];
			A[i].v[1] = A[i].v[1] + A[i].delv[1];
			A[i].w = A[i].w + A[i].delw;
		}

	//	for (i = 0; i < NumberOfObject; i++) for (j = 0; j < NumberOfObject; j++) if (i > j) { positionalcorrection(&A[i], &A[j]); positionalcorrection(&A[j], &A[i]); }
	for (i = 0; i < NumberOfObject; i++) for (j = 0; j < NumberOfObject; j++) if (i > j) { positionalcorrection(&A[i], &A[j]); positionalcorrection(&A[j], &A[i]); }
//	for (i = 0; i < shocknum; i++) shock_propagation(A, NumberOfObject);
//	printf("%f %f %f %f %f\n", A[1].v[0], A[1].delv[0], A[1].delw, A[1].x+A[1].shape->vertex[0][0], A[1].x + A[1].shape->vertex[1][0]);
}

void set_object(Object * A, int shape, int material, double x1, double x2, double v1, double v2, double a1, double a2, double th, double w, double alp, double F1, double F2) {
	A->shape = (Shape *)malloc(sizeof(Shape));
	set_shape(A->shape, shape);
	set_material(A, material);
	set_mass(A);
	A->x[0] = x1; A->x[1] = x2;
	A->v[0] = v1; A->v[1] = v2;
	A->a[0] = a1; A->a[1] = a2;
	A->th = th; A->w = w; A->alp = alp;
	A->F[0] = F1; A->F[1] = F2;
	if (material == 99) A->stable = true;
	else A->stable = false;
}

void set_material(Object * object, int n) {
	enum stuff { rock = 1, wood, metal, ground = 99 };

	if (n == rock) {
		object->material.restitution = 0.1;
		object->material.density = 2.6;
		object->material.us = 0.8 * 20;
		object->material.uk = 0.6 * 20;
		object->material.number = 1;
	}
	else if (n == wood) {
		object->material.restitution = 0.2;
		object->material.density = 0.6;
		object->material.us = 0.4 * 20;
		object->material.uk = 0.2 * 20;
		object->material.number = 2;
	}
	else if (n == metal) {
		object->material.restitution = 0.05;
		object->material.density = 7.874;
		object->material.us = 0.74 * 20;
		object->material.uk = 0.57 * 20;
		object->material.number = 3;
	}
	else if (n == ground) {
		object->material.restitution = 0.1;
		object->material.density = -1;
		object->material.us = 0.8 * 20;
		object->material.uk = 0.6 * 20;
		object->material.number = 99;
	}

}

void set_mass(Object * object) {
	double mass = object->shape->volume*object->material.density;
	object->inv_mass = 1 / mass;
	if (object->material.number == 99) object->inv_mass = 0;
	double inertia = object->shape->inertia_constant*mass*object->shape->volume;
	object->inv_inertia = 1 / inertia;
	if (object->material.number == 99) object->inv_inertia = 0;
}

double find_penetration_depth(Object * A, Object * B, int *indexB, int * indexA) { // find the pedetration depth of B about A: B sink to the A
/*	int i;
	double depth;
	double best_depth = -DBL_MAX;
	double a_vertex[2]; // vertex coordinate o f A about origin
	double b_vertex[2]; // vertex coordinate of B about origin

	for (i = 0; i < A->shape->vertex_num; i++) {
		b_vertex[0] = B->x[0] + B->shape->vertex[*indexB][0];
		b_vertex[1] = B->x[1] + B->shape->vertex[*indexB][1];
		a_vertex[0] = A->x[0] + A->shape->vertex[i][0];
		a_vertex[1] = A->x[1] + A->shape->vertex[i][1];
		depth = v_dot(b_vertex, A->shape->normal[i]) - v_dot(a_vertex, A->shape->normal[i]);
		if (depth > best_depth) {
			best_depth = depth;
			*indexA = i; // i'th normal vector of A
		}

	} */
//	if ((A->shape->max_distance + B->shape->max_distance)*(A->shape->max_distance + B->shape->max_distance) < (A->x[0] - B->x[0])*(A->x[0] - B->x[0]) + (A->x[1] - B->x[1])*(A->x[1] - B->x[1])) return 1;
	if (A->shape->max_distance*A->shape->max_distance < (A->x[0] - B->shape->vertex[*indexB][0] - B->x[0])*(A->x[0] - B->shape->vertex[*indexB][0] - B->x[0]) + (A->x[1] - B->shape->vertex[*indexB][1] - B->x[1])*(A->x[1] - B->shape->vertex[*indexB][1] - B->x[1])) return 1;

	int i,j;
	int index;
	double * depth_array; depth_array = (double *)malloc((A->shape->vertex_num)*sizeof(double));
	double best_depth=-DBL_MAX;
	double a_vertex[2]; // vertex coordinate o f A about origin
	double b_vertex[2]; // vertex coordinate of B about origin

	for (i = 0; i < A->shape->vertex_num; i++) {
		b_vertex[0] = B->x[0] + B->shape->vertex[*indexB][0];
		b_vertex[1] = B->x[1] + B->shape->vertex[*indexB][1];
		a_vertex[0] = A->x[0] + A->shape->vertex[i][0];
		a_vertex[1] = A->x[1] + A->shape->vertex[i][1];
		depth_array[i] = v_dot(b_vertex, A->shape->normal[i]) - v_dot(a_vertex, A->shape->normal[i]);
		}
	
	bool * select;
	select = (bool*)malloc(sizeof(bool)*A->shape->vertex_num);
	for (i = 0; i < A->shape->vertex_num; i++) select[i] = true;

	for (j = 0; j < A->shape->vertex_num; j++) {
		for (i = 0; i < A->shape->vertex_num; i++) if (depth_array[i] > best_depth && select[i] == true) { index = i; best_depth = depth_array[i]; }
		select[index] = false;
		if (best_depth > 0) { free(depth_array); free(select); return best_depth; }
		if (-v_dot(B->shape->vertex[*indexB], A->shape->normal[index]) > 0) { *indexA = index; free(depth_array); free(select); return best_depth; }
		best_depth = -DBL_MAX;
	}

	free(depth_array);
	free(select);
	return best_depth; // positive means there is no collision, negative means there is collision and dept is its absolute value
}

void find_impulse(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA,  int numB) { // Let B sink to the A
	double e = minimum(A->material.restitution, B->material.restitution);
	double impulse; // Impulse magnitude
	double tempt_vector[2]; // temporary vector for calculus
	double VA[2]; // velocity of collision point about A
	double rA[2]; // position vector of collision point from COM of A
	double VB[2]; // velocity of collision point about B
	double rB[2]; // position vector of collision point from COM of B
	double VBA[2]; // VB-VA
	int indexA, indexB;
	int index;

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		if (B->shape->vertex_contact_index[indexB]!=-1 && contact[B->shape->vertex_contact_index[indexB]].numA==numA) {
			index = B->shape->vertex_contact_index[indexB];
			indexA = contact[index].indexA;

			rB[0] = B->shape->vertex[indexB][0]; rB[1] = B->shape->vertex[indexB][1]; // set rB
			v_sum(B->x, rB, tempt_vector); v_sub(tempt_vector, A->x, rA); // set rA
			v_cross_k_inv(A->w+A->delw, rA, tempt_vector); v_sum(tempt_vector, A->v, VA); // set VA
			VA[0] = VA[0] + A->delv[0];	VA[1] = VA[1] + A->delv[1];
			v_cross_k_inv(B->w+B->delw, rB, tempt_vector); v_sum(tempt_vector, B->v, VB); // set VB
			VB[0] = VB[0] + B->delv[0];	VB[1] = VB[1] + B->delv[1];
			v_sub(VB, VA, VBA); // set VBA

			if (v_dot(VBA, A->shape->normal[indexA]) < 0) {
				impulse = (e + 1)*v_dot(A->shape->normal[indexA], VBA);
				impulse = impulse / (A->inv_mass + B->inv_mass + A->inv_inertia*v_cross(rA, A->shape->normal[indexA])*v_cross(rA, A->shape->normal[indexA]) + B->inv_inertia*v_cross(rB, A->shape->normal[indexA])*v_cross(rB, A->shape->normal[indexA])); // set impulse

				v_mult(A->shape->normal[indexA], impulse*A->inv_mass, tempt_vector);
				v_sum(A->delv, tempt_vector, A->delv); // impulse on A's v
				A->delw = A->delw + impulse*A->inv_inertia*v_cross(rA, A->shape->normal[indexA]); // impulse on A's w

				v_mult(A->shape->normal[indexA], impulse*B->inv_mass, tempt_vector);
				v_sub(B->delv, tempt_vector, B->delv); // impulse on B's v
				B->delw = B->delw - impulse*B->inv_inertia*v_cross(rB, A->shape->normal[indexA]); // impulse on B's w

				contact[index].impulse[0] = contact[index].impulse[0] + impulse*A->shape->normal[indexA][0];
				contact[index].impulse[1] = contact[index].impulse[1] + impulse*A->shape->normal[indexA][1];
			}
		}
	}
}

void find_fraction(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB) { // Let B sink to the A
	double u;
	double e = minimum(A->material.restitution, B->material.restitution);
	double impulse; // Impulse magnitude
	double tempt_vector[2]; // temporary vector for calculus
	double VA[2]; // velocity of collision point about A (next value)
	double rA[2]; // position vector of collision point from COM of A
	double VB[2]; // velocity of collision point about B (next value)
	double rB[2]; // position vector of collision point from COM of B
	double VBA[2]; // VB-VA
	int indexA, indexB;
	int index;
	double dt = 1 / fpss;

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		if (B->shape->vertex_contact_index[indexB] != -1 && contact[B->shape->vertex_contact_index[indexB]].numA == numA) {
			index = B->shape->vertex_contact_index[indexB];
			indexA = contact[index].indexA;

			rB[0] = B->shape->vertex[indexB][0]; rB[1] = B->shape->vertex[indexB][1]; // set rB
			v_sum(B->x, rB, tempt_vector); v_sub(tempt_vector, A->x, rA); // set rA
			v_cross_k_inv(A->w + A->delw, rA, tempt_vector); v_sum(tempt_vector, A->v, VA); VA[0] = VA[0] + A->delv[0]; VA[1] = VA[1] + A->delv[1]; // set VA
			v_cross_k_inv(B->w + B->delw, rB, tempt_vector); v_sum(tempt_vector, B->v, VB); VB[0] = VB[0] + B->delv[0]; VB[1] = VB[1] + B->delv[1]; // set VB
			v_sub(VB, VA, VBA); // set VBA
			if (indexA != A->shape->vertex_num - 1) {
				tempt_vector[0] = A->shape->vertex[indexA][0] - A->shape->vertex[indexA + 1][0];
				tempt_vector[1] = A->shape->vertex[indexA][1] - A->shape->vertex[indexA + 1][1];
			}
			else {
				tempt_vector[0] = A->shape->vertex[indexA][0] - A->shape->vertex[0][0];
				tempt_vector[1] = A->shape->vertex[indexA][1] - A->shape->vertex[0][1];
			}
			if (v_dot(VBA, tempt_vector) != 0) {
				impulse = (e + 1)*v_dot(A->shape->normal[indexA], VBA);
				impulse = impulse / (A->inv_mass + B->inv_mass + A->inv_inertia*v_cross(rA, A->shape->normal[indexA])*v_cross(rA, A->shape->normal[indexA]) + B->inv_inertia*v_cross(rB, A->shape->normal[indexA])*v_cross(rB, A->shape->normal[indexA])); // set impulse
				impulse = absolute(impulse);
				find_perpend_v(VBA, A->shape->normal[indexA], tempt_vector);
				v_normalization_s(tempt_vector);
				if (v_magnitude(VBA) > threshold) u = sqrt(A->material.uk*A->material.uk + B->material.uk*B->material.uk);
				else u = sqrt(A->material.us*A->material.us + B->material.us*B->material.us);

				double fraction_magnitude = u*impulse;
				double fraction_vector[2];
				v_mult(tempt_vector, fraction_magnitude, fraction_vector);
				double calculate_tempt[2], calculate_tempt2[2];
				v_cross_k_inv(v_cross(rB, fraction_vector), rB, calculate_tempt);
				v_cross_k_inv(v_cross(rA, fraction_vector), rA, calculate_tempt2);
				if (v_magnitude(VBA) > v_magnitude(fraction_vector)*A->inv_mass + v_magnitude(fraction_vector)*B->inv_mass + v_magnitude(calculate_tempt)*B->inv_inertia + v_magnitude(calculate_tempt2)*A->inv_inertia) {
					double delwB, delwA; // delta value of angular velocity of B and A
					double delvA[2], delvB[2]; // delta value of COM velocity of A and B
					delwB = -dt*v_cross(rB, fraction_vector)*B->inv_inertia;
					delwA = dt*v_cross(rA, fraction_vector)*A->inv_inertia;
					v_mult(fraction_vector, -dt*B->inv_mass, delvB);
					v_mult(fraction_vector, dt*A->inv_mass, delvA);
					A->delw = A->delw + delwA; v_sum(A->delv, delvA, A->delv);
					B->delw = B->delw + delwB; v_sum(B->delv, delvB, B->delv);
				}
				else {
					double P, Q, R;
					P = rB[0] * rB[1] * B->inv_inertia;
					Q = A->inv_mass + B->inv_mass + rB[1] * rB[1] * B->inv_inertia + rA[1] * rA[1] * A->inv_inertia;
					R = A->inv_mass + B->inv_mass + rB[0] * rB[0] * B->inv_inertia + rA[0] * rA[0] * A->inv_inertia;
					fraction_vector[0] = (VBA[0] * R + VBA[1] * P) / (Q*R - P*P);
					fraction_vector[1] = (VBA[0] * P + VBA[1] * Q) / (Q*R - P*P);
					double delwB, delwA; // delta value of angular velocity of B and A
					double delvA[2], delvB[2]; // delta value of COM velocity of A and B
					delwB = -dt*v_cross(rB, fraction_vector)*B->inv_inertia;
					delwA = dt*v_cross(rA, fraction_vector)*A->inv_inertia;
					v_mult(fraction_vector, -dt*B->inv_mass, delvB);
					v_mult(fraction_vector, dt*A->inv_mass, delvA);
					A->delw = A->delw + delwA; v_sum(A->delv, delvA, A->delv);
					B->delw = B->delw + delwB; v_sum(B->delv, delvB, B->delv);
				}

				contact[index].impulse[0] = contact[index].impulse[0] - fraction_vector[0] * dt;
				contact[index].impulse[1] = contact[index].impulse[1] - fraction_vector[1] * dt;
			}
//			}
		}
	}
}

void positionalcorrection(Object * A, Object * B) {
	double tempt_vector[2]; // temporary vector for calculus
	int indexA, indexB;
	double depth;
	const double percent = 0.2;

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		depth = find_penetration_depth(A, B, &indexB, &indexA);
		if (depth < -slop) {
			depth = -depth;
			v_mult(A->shape->normal[indexA], depth*percent / (A->inv_mass + B->inv_mass), tempt_vector);
			double tempttempt_vector[2];
			v_mult(tempt_vector, -A->inv_mass, tempttempt_vector);
			v_sum(A->x, tempttempt_vector, A->x);
			v_mult(tempt_vector, B->inv_mass, tempttempt_vector);
			v_sum(B->x, tempttempt_vector, B->x);
		}
	}
}

void movement(Object * A) {
	double tempt[2];
	double dt = 1 / fpss;
	v_mult(A->F, A->inv_mass, tempt);  // tempt=F/m
	v_mult(tempt, dt, tempt); // tempt=dt * F/m
	v_sum(A->v, tempt, A->v); // v=v+dt*F/m

	v_mult(A->v, dt, tempt); // tempt=v*dt
	v_sum(A->x, tempt, A->x); // x=x+v*dt
	
	A->w = A->w + A->alp*dt;
	A->th = A->th + A->w*dt;
	double rotmat[2][2];
	set_rot_mat(rotmat, A->th);
	int i;
	for (i = 0; i < A->shape->vertex_num; i++) {
		if (A->shape->number == 1) {
			double vertex[4][2];
			vertex[0][0] = Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			double normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);

		}
		else if (A->shape->number == 2) {
			double vertex[4][2];
			vertex[0][0] = Length; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			double normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
		}
		else if (A->shape->number == 3) {
			double vertex[4][2];
			vertex[0][0] = 3 * Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -3 * Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -3 * Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = 3 * Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			double normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
		}
		else if (A->shape->number == 4) {
			double vertex[3][2];
			vertex[0][0] = 0; vertex[0][1] = Length / sqrt(3);
			vertex[1][0] = -Length / 2; vertex[1][1] = -Length / (2 * sqrt(3));
			vertex[2][0] = Length / 2; vertex[2][1] = -Length / (2 * sqrt(3));
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			double normal[3][2];
			normal[0][0] = -sqrt(3) / 2; normal[0][1] = 1 / 2;
			normal[1][0] = 0; normal[1][1] = -1;
			normal[2][0] = sqrt(3) / 2; normal[2][1] = 1 / 2;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
		}
	}
}

void gravity(Object * A) {
	if (A->inv_mass != 0) A->F[1] = A->F[1] + g*(1 / A->inv_mass);
}

void force_zero(Object * A) {
	A->F[0] = 0;
	A->F[1] = 0;
}

void v_sum(double v1[2], double v2[2], double v_out[2]) {
	v_out[0] = v1[0] + v2[0];
	v_out[1] = v1[1] + v2[1];
}

void v_sub(double v1[2], double v2[2], double v_out[2]) { // v_out=v1-v2
	v_out[0] = v1[0] - v2[0];
	v_out[1] = v1[1] - v2[1];
}

double v_dot(double v1[2], double v2[2]) { // v1 * v2
	return v1[0] * v2[0] + v1[1] * v2[1];
}

double v_cross(double v1[2], double v2[2]) { //v1 X v2
	return v1[0] * v2[1] - v1[1] * v2[0];
}

void v_cross_k(double vec[2], double k, double v_out[2]) { // <vec[0], vec[1], 0> X <0,0,k>
	v_out[0] = k*vec[1];
	v_out[1] = -vec[0] * k;
}

void v_cross_k_inv(double k, double vec[2], double v_out[2]) { // <0,0,k> X <vec[0], vec[1], 0>
	v_out[0] = -k*vec[1];
	v_out[1] = vec[0] * k;
}

double v_magnitude(double v[2]) {
	return sqrt(v[0] * v[0] + v[1] * v[1]);
}

void v_normalization(double v_in[2], double v_out[2]) {
	v_out[0] = v_in[0] / v_magnitude(v_in);
	v_out[1] = v_in[1] / v_magnitude(v_in);
}

void v_normalization_s(double v_in[2]) {
	double mag = v_magnitude(v_in);
	v_in[0] = v_in[0] / mag;
	v_in[1] = v_in[1] / mag;
}

void mat_v_product(double mat[2][2], double v[2], double v_out[2]) { // matrix * vector
	v_out[0] = mat[0][0] * v[0] + mat[0][1] * v[1];
	v_out[1] = mat[1][0] * v[0] + mat[1][1] * v[1];
}

void set_rot_mat(double mat[2][2], double radius) {
	mat[0][0] = cos(radius);
	mat[0][1] = -sin(radius);
	mat[1][0] = sin(radius);
	mat[1][1] = cos(radius);
}

void find_normal_v(double v[2], double dir[2], double v_out[2]) { // find vector along some direction
	double magnitude;
	double normalization_vector[2];
	v_normalization(dir, normalization_vector);
	magnitude = v_dot(v, normalization_vector);
	v_out[0] = magnitude*normalization_vector[0];
	v_out[1] = magnitude*normalization_vector[1];
}

void find_perpend_v(double v[2], double dir[2], double v_out[2]) { // find vector perpendicular to some direction
	double normal_vector[2];
	find_normal_v(v, dir, normal_vector);
	v_out[0] = v[0] - normal_vector[0];
	v_out[1] = v[1] - normal_vector[1];
}

void v_mult(double v[2], double n, double v_out[2]) {
	v_out[0] = n*v[0];
	v_out[1] = n*v[1];
}

double minimum(double a, double b) {
	if (a >= b) return b;
	else return a;
}

double maximum(double a, double b) {
	if (a <= b) return b;
	else return a;
}

double absolute(double a) {
	if (a >= 0) return a;
	else return -a;
}

void set_shape(Shape * shape, int n) {
	enum polygon { oneblock = 1, twoblock = 2, threeblock = 3, triangle=4, ground = 99 };
	if (n == oneblock) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (double *)malloc(sizeof(double) * 2);
		shape->vertex[0][0] = Length / 2; shape->vertex[0][1] = Length / 2;
		shape->vertex[1][0] = -Length / 2; shape->vertex[1][1] = Length / 2;
		shape->vertex[2][0] = -Length / 2; shape->vertex[2][1] = -Length / 2;
		shape->vertex[3][0] = Length / 2; shape->vertex[3][1] = -Length / 2;

		shape->normal = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (double *)malloc(sizeof(double) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = Length*Length;
		shape->inertia_constant = 1 / 6.0;
		shape->number = 1;

		shape->max_distance = Length / sqrt(2);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;
		/*
		n[0]
		|
		vert[1]----vert[0]
		|				|
		|				|
		n[1]-|				|-n[3]
		|				|
		vert[2]----vert[3]
		|
		n[2]
		*/
	}
	else if (n == twoblock) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (double *)malloc(sizeof(double) * 2);
		shape->vertex[0][0] = Length; shape->vertex[0][1] = Length / 2;
		shape->vertex[1][0] = -Length; shape->vertex[1][1] = Length / 2;
		shape->vertex[2][0] = -Length; shape->vertex[2][1] = -Length / 2;
		shape->vertex[3][0] = Length; shape->vertex[3][1] = -Length / 2;

		shape->normal = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (double *)malloc(sizeof(double) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = 2 * Length*Length;
		shape->inertia_constant = 5 / 24.0;
		shape->number = 2;

		shape->max_distance = 0.5*Length*sqrt(5);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;
		/*
					n[0]
					|
		vert[1]-----------------vert[0]
		|			|			|
		|			|			|
   n[1]-|			|			|-n[3]
		|			|			|
		vert[2]-----------------vert[3]
					|
					n[2]
		*/
	}
	else if (n == threeblock) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (double *)malloc(sizeof(double) * 2);
		shape->vertex[0][0] = 3 * Length / 2; shape->vertex[0][1] = Length / 2;
		shape->vertex[1][0] = -3 * Length / 2; shape->vertex[1][1] = Length / 2;
		shape->vertex[2][0] = -3 * Length / 2; shape->vertex[2][1] = -Length / 2;
		shape->vertex[3][0] = 3 * Length / 2; shape->vertex[3][1] = -Length / 2;

		shape->normal = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (double *)malloc(sizeof(double) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = 3 * Length*Length;
		shape->inertia_constant = 5 / 18.0;
		shape->number = 3;

		shape->max_distance = 0.5*Length*sqrt(10);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;
		/*
		n[0]
		|
		vert[1]-------------------------vert[0]
		|			|			|			|
		|			|			|			|
		n[1]-|			|			|			|-n[3]
		|			|			|			|
		vert[2]-------------------------vert[3]
		|
		n[2]
		*/
	}
	else if (n == triangle) {
		int i;
		shape->vertex_num = 3;
		shape->vertex = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (double *)malloc(sizeof(double) * 2);
		shape->vertex[0][0] = 0; shape->vertex[0][1] = Length/sqrt(3);
		shape->vertex[1][0] = -Length / 2; shape->vertex[1][1] = -Length / (2 * sqrt(3));
		shape->vertex[2][0] = Length / 2; shape->vertex[2][1] = -Length / (2 * sqrt(3));

		shape->normal = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (double *)malloc(sizeof(double) * 2);
		shape->normal[0][0] = -sqrt(3) / 2; shape->normal[0][1] = 1 / 2;
		shape->normal[1][0] = 0; shape->normal[1][1] = -1;
		shape->normal[2][0] = sqrt(3) / 2; shape->normal[2][1] = 1 / 2;

		shape->volume = sqrt(3) * Length*Length / 4;
		shape->inertia_constant = sqrt(3) / 9.0;
		shape->number = 4;

		shape->max_distance = Length / sqrt(3);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;
		/*
		                vert[0]
	                  /        \    n[2]
		    n[0]     /	        \  /
				 \  /            \
                   /              \
				vert[1]--------vert[2]
				          |
				         n[1]
		*/
	}
	else if (n == ground) {
		int i;
		shape->vertex_num = 4;
		shape->vertex = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex[i] = (double *)malloc(sizeof(double) * 2);
		shape->vertex[0][0] = 7 * Length; shape->vertex[0][1] = 2*Length;
		shape->vertex[1][0] = -7 * Length; shape->vertex[1][1] = 2*Length;
		shape->vertex[2][0] = -7 * Length; shape->vertex[2][1] = -2*Length;
		shape->vertex[3][0] = 7 * Length; shape->vertex[3][1] = -2*Length;

		shape->normal = (double **)malloc(sizeof(double *)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->normal[i] = (double *)malloc(sizeof(double) * 2);
		shape->normal[0][0] = 0; shape->normal[0][1] = 1;
		shape->normal[1][0] = -1; shape->normal[1][1] = 0;
		shape->normal[2][0] = 0; shape->normal[2][1] = -1;
		shape->normal[3][0] = 1; shape->normal[3][1] = 0;

		shape->volume = 56 * Length*Length;
		shape->inertia_constant = 5 / 18.0;
		shape->number = 99;

		shape->max_distance = Length*sqrt(53);

		shape->vertex_contact_index = (int *)malloc(sizeof(int)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact_index[i] = -1;

		shape->vertex_contact = (bool *)malloc(sizeof(bool)*shape->vertex_num);
		for (i = 0; i < shape->vertex_num; i++) shape->vertex_contact[i] = false;
		/*
		n[0]
		|
		vert[1]-------------------------vert[0]
		|			|			|			|
		|			|			|			|
		n[1]-|			|			|			|-n[3]
		|			|			|			|
		vert[2]-------------------------vert[3]
		|
		n[2]
		*/
	}
};

double click_perception(int mouse_x, int mouse_y, Object * A) { // find the pedetration depth of B about A: B sink to the A
	int i;
	double depth;
	double best_depth = -DBL_MAX;
	double a_vertex[2]; // vertex coordinate o f A about origin
	double b_vertex[2]; // vertex coordinate of B about origin
	for (i = 0; i < A->shape->vertex_num; i++) {
		b_vertex[0] = mouse_x;
		b_vertex[1] = mouse_y;
		a_vertex[0] = A->x[0] + A->shape->vertex[i][0];
		a_vertex[1] = A->x[1] + A->shape->vertex[i][1];
		depth = v_dot(b_vertex, A->shape->normal[i]) - v_dot(a_vertex, A->shape->normal[i]);
		if (depth > best_depth) best_depth = depth;
	}
	return best_depth; // positive means there is click, negative means there is click and dept is its absolute value
}

void reassign_vertex(Object * A) {
	double tempt[2];

	double rotmat[2][2];
	set_rot_mat(rotmat, A->th);
	int i;
	for (i = 0; i < A->shape->vertex_num; i++) {
		if (A->shape->number == 1) {
			double vertex[4][2];
			vertex[0][0] = Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			double normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);

		}
		else if (A->shape->number == 2) {
			double vertex[4][2];
			vertex[0][0] = Length; vertex[0][1] = Length / 2;
			vertex[1][0] = -Length; vertex[1][1] = Length / 2;
			vertex[2][0] = -Length; vertex[2][1] = -Length / 2;
			vertex[3][0] = Length; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			double normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
		}
		else if (A->shape->number == 3) {
			double vertex[4][2];
			vertex[0][0] = 3 * Length / 2; vertex[0][1] = Length / 2;
			vertex[1][0] = -3 * Length / 2; vertex[1][1] = Length / 2;
			vertex[2][0] = -3 * Length / 2; vertex[2][1] = -Length / 2;
			vertex[3][0] = 3 * Length / 2; vertex[3][1] = -Length / 2;
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			mat_v_product(rotmat, vertex[3], A->shape->vertex[3]);
			double normal[4][2];
			normal[0][0] = 0; normal[0][1] = 1;
			normal[1][0] = -1; normal[1][1] = 0;
			normal[2][0] = 0; normal[2][1] = -1;
			normal[3][0] = 1; normal[3][1] = 0;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
			mat_v_product(rotmat, normal[3], A->shape->normal[3]);
		}
		else if (A->shape->number == 4) {
			double vertex[3][2];
			vertex[0][0] = 0; vertex[0][1] = Length / sqrt(3);
			vertex[1][0] = -Length / 2; vertex[1][1] = -Length / (2 * sqrt(3));
			vertex[2][0] = Length / 2; vertex[2][1] = -Length / (2 * sqrt(3));
			mat_v_product(rotmat, vertex[0], A->shape->vertex[0]);
			mat_v_product(rotmat, vertex[1], A->shape->vertex[1]);
			mat_v_product(rotmat, vertex[2], A->shape->vertex[2]);
			double normal[3][2];
			normal[0][0] = -sqrt(3) / 2; normal[0][1] = 1 / 2;
			normal[1][0] = 0; normal[1][1] = -1;
			normal[2][0] = sqrt(3) / 2; normal[2][1] = 1 / 2;
			mat_v_product(rotmat, normal[0], A->shape->normal[0]);
			mat_v_product(rotmat, normal[1], A->shape->normal[1]);
			mat_v_product(rotmat, normal[2], A->shape->normal[2]);
		}
	}
}

void delta_zero(Object * A) {
	A->delv[0] = 0;
	A->delv[1] = 0;
	A->delw = 0;
}

void shock_propagation(Object A[], int NumberOfObject) {
	int i;

	int index;
	int indexA, indexB;

	double best_y_value=-DBL_MAX;
	double last_y_value = DBL_MAX;

	bool loop_escape;

	bool contact = false;

	bool * calculate_end;
	calculate_end = (bool *)malloc(sizeof(bool)*NumberOfObject);

	for (i = 0; i < NumberOfObject; i++) calculate_end[i] = false;

	for (i = 0; i < NumberOfObject; i++) {
		stability_zero(&A[i]);
		A[i].delv[0] = 0;
		A[i].delv[1] = 0;
		A[i].delw = 0;
	}

	loop_escape = false;
	while (!loop_escape) {
		loop_escape = true;
		for (i = 0; i < NumberOfObject; i++) {
			if (A[i].x[1] >= best_y_value && calculate_end[i] == false && A[i].x[1] <= last_y_value && A[i].stable==false) { best_y_value = A[i].x[1]; index = i; loop_escape = false; }
		}
		if (loop_escape == true) break;
		/*---*/
		for (i = 0; i < NumberOfObject; i++) if (i != index && A[i].stable==true) {
				find_shock(&A[index], &A[i], &contact);
				find_shock(&A[i], &A[index], &contact);
		}
		if (contact == true) { contact = false; A[index].stable = true; }
		/*---*/
		calculate_end[index] = true;
		last_y_value = best_y_value;
		best_y_value = -DBL_MAX;
	}
	free(calculate_end);
	for (i = 0; i < NumberOfObject; i++) {
		A[i].v[0] = A[i].v[0] + A[i].delv[0];
		A[i].v[1] = A[i].v[1] + A[i].delv[1];
		A[i].w = A[i].w + A[i].delw;
	}
}

void find_shock(Object * A, Object * B, bool * contact) { // Let B sink to the A
	double e = minimum(A->material.restitution, B->material.restitution);
	double impulse; // Impulse magnitude
	double tempt_vector[2]; // temporary vector for calculus
	double VA[2]; // velocity of collision point about A
	double rA[2]; // position vector of collision point from COM of A
	double VB[2]; // velocity of collision point about B
	double rB[2]; // position vector of collision point from COM of B
	double VBA[2]; // VB-VA
	double imaginary_inv_mass_A = A->inv_mass;
	double imaginary_inv_mass_B = B->inv_mass;
	double imaginary_inv_inertia_A = A->inv_inertia;
	double imaginary_inv_inertia_B = B->inv_inertia;
	int indexA, indexB;

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		if (find_penetration_depth(A, B, &indexB, &indexA) <= 0) {
			*contact = true;
			rB[0] = B->shape->vertex[indexB][0]; rB[1] = B->shape->vertex[indexB][1]; // set rB
			v_sum(B->x, rB, tempt_vector); v_sub(tempt_vector, A->x, rA); // set rA
			v_cross_k_inv(A->w, rA, tempt_vector); v_sum(tempt_vector, A->v, VA); // set VA
			v_cross_k_inv(B->w, rB, tempt_vector); v_sum(tempt_vector, B->v, VB); // set VB
			v_sub(VB, VA, VBA); // set VBA
			if (v_dot(VBA, A->shape->normal[indexA]) < 0) {
				if (A->stable == true) { imaginary_inv_mass_A = 0; imaginary_inv_inertia_A = 0; }
				if (B->stable == true) { imaginary_inv_mass_B = 0; imaginary_inv_inertia_B = 0; }
				impulse = (e + 1)*v_dot(A->shape->normal[indexA], VBA)*(1.0/shocknum);
				impulse = impulse / (imaginary_inv_mass_A + imaginary_inv_mass_B + imaginary_inv_inertia_A*v_cross(rA, A->shape->normal[indexA])*v_cross(rA, A->shape->normal[indexA]) + imaginary_inv_inertia_B*v_cross(rB, A->shape->normal[indexA])*v_cross(rB, A->shape->normal[indexA])); // set impulse

				v_mult(A->shape->normal[indexA], impulse*imaginary_inv_mass_A, tempt_vector);
				v_sum(A->delv, tempt_vector, A->delv); // impulse on A's v
				A->delw = A->delw + impulse*imaginary_inv_inertia_A*v_cross(rA, A->shape->normal[indexA]); // impulse on A's w

				v_mult(A->shape->normal[indexA], impulse*imaginary_inv_mass_B, tempt_vector);
				v_sub(B->delv, tempt_vector, B->delv); // impulse on B's v
				B->delw = B->delw - impulse*imaginary_inv_inertia_B*v_cross(rB, A->shape->normal[indexA]); // impulse on B's w

			}
		}
	}
}


void stability_zero(Object * A) {
	int i;
	if (A->material.number != 99) A->stable = false;
	else A->stable= true;
}

void collision_check(Object * A, Object * B, int NumberOfObject, cinf contact[], int numA, int numB) { // Let B sink to the A
	double tempt_vector[2]; // temporary vector for calculus
	double rA[2]; // position vector of collision point from COM of A
	double rB[2]; // position vector of collision point from COM of B
	double VBA[2]; // VB-VA
	int indexA, indexB;
	int i;
	int index;
	double depth;
	double drA[2], drB[2];

	for (indexB = 0; indexB < B->shape->vertex_num; indexB++) {
		depth = find_penetration_depth(A, B, &indexB, &indexA);
		if (depth <= 0) {
			B->shape->vertex_contact[indexB] = true;
			rB[0] = B->shape->vertex[indexB][0]; rB[1] = B->shape->vertex[indexB][1]; // set rB
			v_sum(B->x, rB, tempt_vector); v_sub(tempt_vector, A->x, rA); // set rA
				if (B->shape->vertex_contact_index[indexB] == -1) {
					for (i = 0; i < 4 * NumberOfObject; i++) if (contact[i].run == false) {
						index = i;
						contact[index].run=true;
						contact[index].numA=numA;
						contact[index].numB=numB;
						contact[index].indexA=indexA;
						contact[index].indexB=indexB;
						contact[index].rA[0] = rA[0]; contact[index].rA[1] = rA[1];
						contact[index].rB[0] = rB[0]; contact[index].rB[1] = rB[1];
						contact[index].depth = depth;
						B->shape->vertex_contact_index[indexB] = index;
						break;
					}
				}
				else {
					index = B->shape->vertex_contact_index[indexB];
					drA[0] = rA[0] - contact[index].rA[0]; drA[1] = rA[1] - contact[index].rA[1];
					drB[0] = rB[0] - contact[index].rB[0]; drB[1] = rB[1] - contact[index].rB[1];
					if (contact[index].numA == numA && contact[index].indexA == indexA && drA[0] * drA[0] + drA[1] * drA[1] < warmth&& drB[0] * drB[0] + drB[1] * drB[1] < warmth) {
						contact[index].run = true;
//						contact[index].numA = numA;
//						contact[index].numB = numB;
//						contact[index].indexA = indexA;
//						contact[index].indexB = indexB;
//						contact[index].rA[0] = rA[0]; contact[index].rA[1] = rA[1];
//						contact[index].rB[0] = rB[0]; contact[index].rB[1] = rB[1];
//						contact[index].depth = depth;
					}
					else {
						contact[index].run = true;
						contact[index].numA = numA;
						contact[index].numB = numB;
						contact[index].indexA = indexA;
						contact[index].indexB = indexB;
						contact[index].rA[0] = rA[0]; contact[index].rA[1] = rA[1];
						contact[index].rB[0] = rB[0]; contact[index].rB[1] = rB[1];
						contact[index].depth = depth;
						contact[index].impulse[0] = 0;
						contact[index].impulse[1] = 0;
					}
				}
		}
	}
}

void uncollide_check(Object A[], int NumberOfObject, cinf contact[]) {
	int i, j;
	int index;
	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < A[i].shape->vertex_num;j++) {
			if (A[i].shape->vertex_contact[j] == false && A[i].shape->vertex_contact_index[j] != -1) {
				index = A[i].shape->vertex_contact_index[j];
				contact[index].run = false;
				contact[index].numA = 0;
				contact[index].numB = 0;
				contact[index].indexA = 0;
				contact[index].indexB = 0;
				contact[index].rA[0] = 0;
				contact[index].rA[1] = 0;
				contact[index].rB[0] = 0;
				contact[index].rB[1] = 0;
				contact[index].depth = 0;
				contact[index].impulse[0] = 0;
				contact[index].impulse[1] = 0;
				A[i].shape->vertex_contact_index[j] = -1;
			}
		}
	}
	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < A[i].shape->vertex_num; j++) {
			A[i].shape->vertex_contact[j] = false;
		}
	}
}

void warmstarting(Object A[], int NumberOfObject, cinf contact[]) {
	int i, j;
	int index, indexA;
	int numA, numB;
	for (i = 0; i < NumberOfObject; i++) {
		for (j = 0; j < A[i].shape->vertex_num; j++) {
			if (A[i].shape->vertex_contact_index[j] != -1) {
				index = A[i].shape->vertex_contact_index[j];
				numA = contact[index].numA;
				numB = contact[index].numB;
				indexA = contact[index].indexA;
				if (v_dot(contact[index].impulse, A[numA].shape->normal[indexA]) > 0) { contact[index].impulse[0] = 0; contact[index].impulse[1] = 0; }
				A[numB].delv[0] = A[numB].delv[0] - contact[index].impulse[0] * A[numB].inv_mass;
				A[numB].delv[1] = A[numB].delv[1] - contact[index].impulse[1] * A[numB].inv_mass;
				A[numA].delv[0] = A[numA].delv[0] + contact[index].impulse[0] * A[numA].inv_mass;
				A[numA].delv[1] = A[numA].delv[1] + contact[index].impulse[1] * A[numA].inv_mass;
				A[numB].delw = A[numB].delw - v_cross(contact[index].rB, contact[index].impulse)* A[numB].inv_inertia;
				A[numA].delw = A[numA].delw + v_cross(contact[index].rA, contact[index].impulse)* A[numA].inv_inertia;
				contact[index].impulse[0] = 0;
				contact[index].impulse[1] = 0;
			}
		}
	}
}