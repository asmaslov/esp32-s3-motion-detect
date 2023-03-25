/* Generated by Edge Impulse
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_
#define _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_

#include "edge-impulse-sdk/anomaly/anomaly.h"

// (before - mean) / scale
const float ei_classifier_anom_scale[EI_CLASSIFIER_ANOM_AXIS_SIZE] = { 183.48194752638304, 0.881795971323112, 1.0928593860767069, 1.1555823930215552, 0.6914535837318366, 0.8659195083673862, 0.9722984037720872, 0.7590649970635106, 0.7817239838570416, 0.8678086407853581 };
const float ei_classifier_anom_mean[EI_CLASSIFIER_ANOM_AXIS_SIZE] = { 90.33425571735089, 2.900817674627671, 3.0639714354505907, 2.6787947198232778, 2.8504611207888675, 2.9603958974664026, 2.553492323710368, 3.3087580057290884, 2.8373580234555096, 2.6031227563436214 };


const ei_classifier_anom_cluster_t ei_classifier_anom_clusters[EI_CLASSIFIER_ANOM_CLUSTER_COUNT] = { { { 2.1008810997009277, 2.201932430267334, 2.1965832710266113, 2.296685218811035, 0.6142704486846924, 1.8691747188568115, 2.048738718032837, 1.0140193700790405, 1.700542688369751, 2.128312349319458 }, 1.9792115617119308 }
, { { -0.2537868618965149, 0.24648231267929077, 0.20123496651649475, 0.22142988443374634, -0.6897525191307068, 0.3488302528858185, 0.2660801410675049, 0.3302746117115021, 0.2974386513233185, 0.007560469675809145 }, 2.0712172971113647 }
, { { -0.333421528339386, -0.17870277166366577, -0.0883159190416336, -0.7753422260284424, -0.38845884799957275, -0.20369911193847656, -0.5072543621063232, 0.5209658741950989, -0.015539649873971939, 0.11979648470878601 }, 1.5247176300587235 }
, { { -0.3191966712474823, -0.1599491983652115, -0.05376884341239929, -0.3040739893913269, 0.3193148672580719, 0.07266609370708466, -0.38694310188293457, 0.34504345059394836, 0.6486693024635315, 0.15353436768054962 }, 1.4037937439479067 }
, { { -0.4139291048049927, -0.937670111656189, -1.2183315753936768, -1.2604137659072876, -0.7404336333274841, -2.0456743240356445, -0.6664004325866699, -1.0409013032913208, -1.0237963199615479, -1.0167465209960938 }, 1.8662306061628828 }
, { { -0.41408368945121765, -1.6496899127960205, -0.8259572982788086, -0.7758488059043884, -1.3814584016799927, -0.8812189698219299, -0.9528350830078125, -1.289913296699524, -0.8442760705947876, -1.7441736459732056 }, 1.7901968315822188 }
, { { -0.3042163550853729, 0.1571643054485321, 0.11806793510913849, -0.08972369134426117, 0.26940658688545227, 0.4169815182685852, 0.06168360635638237, 0.3193604648113251, -0.08597079664468765, -0.6216274499893188 }, 1.430664284772464 }
, { { -0.145257830619812, 0.27960601449012756, 0.12508085370063782, 0.30462247133255005, 1.0409879684448242, 0.7298062443733215, 0.7741254568099976, 0.9448307156562805, 1.218637228012085, 0.6414526104927063 }, 1.9263140641958723 }
, { { -0.29297924041748047, -0.30018365383148193, 0.12394598126411438, -0.12762369215488434, 0.6201013326644897, 0.45265740156173706, 0.2273440957069397, 0.336311012506485, -0.15576118230819702, 0.32191383838653564 }, 1.4088885532437574 }
, { { -0.06474925577640533, 0.4737473130226135, 0.7822720408439636, 0.7429068088531494, -0.08058131486177444, 0.6954818367958069, 0.7636507749557495, 0.6905517578125, 1.0004123449325562, 0.8245294094085693 }, 1.4858402401563735 }
, { { -0.33110299706459045, -0.2010057270526886, -0.27478930354118347, -0.19542180001735687, 0.011063504964113235, -0.7181142568588257, -0.478951096534729, 0.5486642718315125, 0.16415822505950928, -0.06266654282808304 }, 2.0999483310035254 }
, { { -0.22086845338344574, 0.5127792358398438, 0.4008016586303711, 0.25982534885406494, 0.37653642892837524, 0.29699623584747314, -0.0832219049334526, 0.5601757764816284, 0.47933343052864075, 0.4319803714752197 }, 1.340565021174478 }
, { { 0.4372713267803192, 1.4270192384719849, 1.2433146238327026, 1.3733241558074951, 1.105558156967163, 0.5317664742469788, 1.0689172744750977, 0.5757547616958618, 0.5630781054496765, 0.8231160640716553 }, 1.6635200267076686 }
, { { -0.42321330308914185, -0.9272621870040894, -1.289243221282959, -0.9916617274284363, -1.8391236066818237, -0.9246373772621155, -1.022480845451355, -1.3013752698898315, -1.2757784128189087, -0.7553822994232178 }, 1.8836740849625206 }
, { { -0.3176480829715729, 0.17733632028102875, -0.010134465992450714, 0.06401446461677551, 0.21086400747299194, -0.1960929036140442, -0.41889306902885437, 0.13028058409690857, -1.0925360918045044, 0.26062241196632385 }, 1.8615028185644995 }
, { { -0.41371434926986694, -0.9650488495826721, -1.2070966958999634, -0.879982590675354, -0.6737832427024841, -0.8923512101173401, -1.0010393857955933, -1.9022895097732544, -0.5843826532363892, -0.7558282017707825 }, 1.6159810430465362 }
, { { -0.3449173867702484, -0.5512425899505615, -0.25510355830192566, -0.42949163913726807, -0.29835790395736694, 0.24675318598747253, -0.35817500948905945, -0.3306128978729248, -0.4094393253326416, -0.38210800290107727 }, 1.8272644996996505 }
, { { 0.7800717949867249, 1.9739166498184204, 1.2907092571258545, 1.459578037261963, 1.4953500032424927, 1.8298286199569702, 1.8394064903259277, 1.3013163805007935, 0.7134718894958496, 1.5294804573059082 }, 1.3398419875534973 }
, { { -0.41670942306518555, -1.3031134605407715, -1.0830104351043701, -0.7299593091011047, -0.622948169708252, -1.0676389932632446, -0.8092038631439209, -1.1159034967422485, -1.3079081773757935, -0.9572148323059082 }, 1.3569999258527148 }
, { { -0.0191537756472826, 0.3401481509208679, 0.9762717485427856, 0.6002557873725891, 0.5570971965789795, 1.1429013013839722, 0.5914043188095093, 0.7037115097045898, 0.009699082002043724, 0.8947033882141113 }, 1.2871161766739574 }
, { { -0.41484299302101135, -0.52861088514328, -0.855552613735199, -0.8676890134811401, -0.9179195165634155, -1.1872758865356445, -0.8325367569923401, -1.5884902477264404, -1.2638249397277832, -1.2868597507476807 }, 1.7112414889241507 }
, { { 5.094668865203857, 2.1063990592956543, 2.2963449954986572, 2.955226421356201, 1.5497682094573975, 2.3901071548461914, 2.897057294845581, 1.4242401123046875, 2.651747941970825, 2.4417495727539062 }, 1.8327711756539709 }
, { { 1.2132666110992432, 1.3567520380020142, 1.6981898546218872, 1.7589976787567139, 1.2503814697265625, 1.5566418170928955, 1.827205777168274, 0.7791064977645874, 1.4306610822677612, 1.4363595247268677 }, 1.58892704721814 }
, { { -0.35084179043769836, -0.5661230683326721, -0.3521707355976105, -0.2989339232444763, 0.5041394233703613, -0.027814021334052086, -0.2789387106895447, -0.27680888772010803, 0.14708130061626434, -0.5582775473594666 }, 1.4525941083310863 }
, { { 2.8453030586242676, 2.6536686420440674, 2.2790040969848633, 2.2951889038085938, 2.864837169647217, 2.1745407581329346, 2.3169784545898438, 1.4437354803085327, 1.9880183935165405, 1.534691333770752 }, 1.9930666098094802 }
, { { 3.444465398788452, 2.4547510147094727, 2.091428756713867, 2.6146767139434814, 1.8063418865203857, 1.9009701013565063, 2.532879114151001, 1.6532893180847168, 1.61650550365448, 2.2482264041900635 }, 1.5522566396119755 }
, { { -0.4134104251861572, -0.41328689455986023, -1.1764955520629883, -0.9627853631973267, -0.8409756422042847, -0.9003639817237854, -0.6933799386024475, -1.586181879043579, -2.17572021484375, -0.868506133556366 }, 1.76633467589757 }
, { { 3.4500350952148438, 2.575762987136841, 2.238119602203369, 2.6033098697662354, 1.7700477838516235, 1.0098588466644287, 2.3378655910491943, -0.09026572853326797, 1.994735836982727, 2.433285713195801 }, 0.7406974363429024 }
, { { -0.06617297232151031, 0.28964921832084656, 0.7713508605957031, 0.5574133992195129, 1.175301194190979, 0.5966818928718567, 0.7943657040596008, 1.1393775939941406, 0.8791950345039368, -0.8755923509597778 }, 1.3579986858004651 }
, { { -0.20705513656139374, 0.6722835898399353, 0.5611407160758972, 0.24809753894805908, 0.18421247601509094, 0.5320530533790588, 0.35898974537849426, -0.990637481212616, -0.3369811475276947, 0.5252824425697327 }, 1.7053804193330744 }
, { { 4.278282165527344, 2.4326109886169434, 2.4652976989746094, 2.5957067012786865, 1.5287344455718994, 1.140222430229187, 2.764177083969116, 1.1364878416061401, 1.0745662450790405, 2.2271180152893066 }, 1.0735603281709547 }
, { { 1.5347980260849, 1.4341756105422974, 2.164396047592163, 2.1136884689331055, 2.233905553817749, 1.6290584802627563, 2.1107280254364014, 1.0491055250167847, 1.072771430015564, 1.9549349546432495 }, 1.2948464887915543 }
};


#endif // _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_