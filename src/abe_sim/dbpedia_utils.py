import re
import urllib

from SPARQLWrapper import SPARQLWrapper, JSON

SPARQL_PERISHABLE_FOOD_WHERE_CLAUSE = """
    {
    ?thing rdf:type dbo:Food .
    ?thing dbo:ingredient ?ingredient .
    }
    UNION
    {
    ?thing dcterms:subject dbc:Edible_fruits .
    }
    UNION
    {
    ?thing dcterms:subject dbc:Fruit_vegetables .
    }
    UNION
    {
    ?thing dcterms:subject dbc:Root_vegetables .
    }
    """

SPARQL_VESSEL_WHERE_CLAUSE = """
    {
        ?thing dcterms:subject dbc:Cookware_and_bakeware .
    }
    UNION
    {
        ?thing dcterms:subject dbc:Cooking_vessels .
    }"""

SPARQL_UTENSIL_WHERE_CLAUSE = """
  {
    ?thing gold:hypernym dbr:Utensil .
  }
  UNION
  {
    ?thing dcterms:subject dbc:Food_preparation_utensils .
  }
  UNION
  {
    ?thing dcterms:subject dbc:Eating_utensils .
  }
  """


def extract_info_from_dbpedia(where_clause: str):
    # Define the SPARQL endpoint
    sparql = SPARQLWrapper("https://dbpedia.org/sparql")
    # Set the SPARQL query
    sparql_query = f"""
    PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
    PREFIX gold: <http://purl.org/linguistics/gold/>
    PREFIX dcterms: <http://purl.org/dc/terms/>
    PREFIX dbo: <http://dbpedia.org/ontology/>
    PREFIX dbc: <http://dbpedia.org/resource/Category:>
    SELECT DISTINCT ?thing
    WHERE {{
    {where_clause}
    }}

    """
    # Set the SPARQL query
    sparql.setQuery(sparql_query)
    # Set the return format to JSON
    sparql.setReturnFormat(JSON)
    # Execute the query and store the results
    results = sparql.query().convert()
    # Extract the values from the results and store them in a list
    result_set = set()
    for result in results["results"]["bindings"]:
        food_uri = result["thing"]["value"]
        food_name = process_result_from_dbpedia(food_uri)
        result_set.add(food_name)

    return result_set


def process_result_from_dbpedia(dbpedia_uri):
    individual = urllib.parse.unquote(dbpedia_uri.split('/')[-1]).lower()  # Extract the name from the URI
    individual = re.sub(r'\([^)]*\)', '', individual)  # Remove parentheses and their contents
    individual = re.sub('_+', '_', individual)  # Remove multiple underscores
    individual = re.sub('_\\d+', '', individual)  # Remove substrings like "_1"
    individual = re.sub(r'\b(\w+)(?:_\1)+\b', r'\1', individual)  # Remove consecutive duplicate words,
    # for e.g. transform molten_chocolate_cake_molten_chocolate_cake into molten_chocolate_cake
    return individual.strip('_')


DBPEDIA_PERISHABLE_FOODS = extract_info_from_dbpedia(SPARQL_PERISHABLE_FOOD_WHERE_CLAUSE)
DBPEDIA_VESSELS = extract_info_from_dbpedia(SPARQL_VESSEL_WHERE_CLAUSE)
DBPEDIA_UTENSILS = extract_info_from_dbpedia(SPARQL_UTENSIL_WHERE_CLAUSE)
