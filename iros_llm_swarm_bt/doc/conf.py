project = 'iros_llm_swarm_bt'
html_title = project
master_doc = 'index'
exclude_patterns = []
extensions = []

# Prepend a back-link to the workspace aggregator on every page.
rst_prolog = """
.. raw:: html

   <p style="margin: 0 0 1em 0;">
     <a href="../iros_llm_swarm_docs/index.html">&laquo; IROS LLM Swarm — back to overview</a>
   </p>
"""
